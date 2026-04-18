#include "foc_bridge.h"
#include "PMSM_Control_Core/Clarke_Park.h"
#include "PMSM_Control_Core/FluxObserver_PLL.h"
#include "PMSM_Control_Core/PI_Controller.h"
#include <math.h>

/* FOC 桥接层。
 *
 * 这一层更像“胶水代码”：
 * - 上面连着中断、PWM、ADC 和 GPIO；
 * - 下面连着开环控制和 observer；
 * - 中间负责把采样、状态机、保护和调试串成一条稳定的路径。
 *
 * 调试时最重要的不是每一项都很聪明，而是每一项都足够可解释。
 * 所以这里的实现优先保证：
 * - 顺序清晰；
 * - 状态明确；
 * - 便于串口打印和上位机观察。
 */

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

static volatile FOC_StateTypeDef s_state = FOC_STATE_IDLE;           /* 当前 FOC 状态机状态。 */
static volatile float s_iu_offset_v = 0.0f;                          /* U 相电流采样零点电压。 */
static volatile float s_iv_offset_v = 0.0f;                          /* V 相电流采样零点电压。 */
static volatile float s_iw_offset_v = 0.0f;                          /* W 相电流采样零点电压。 */
static volatile float s_vbus_v = 0.0f;                               /* 母线电压，单位伏。 */
static volatile float s_iu_a = 0.0f;                                 /* U 相电流，单位安。 */
static volatile float s_iv_a = 0.0f;                                 /* V 相电流，单位安。 */
static volatile float s_iw_a = 0.0f;                                 /* W 相电流，单位安。 */
static volatile uint16_t s_raw_u = 0u;                               /* U 相 ADC 注入通道原始值。 */
static volatile uint16_t s_raw_v = 0u;                               /* V 相 ADC 注入通道原始值。 */
static volatile uint16_t s_raw_w = 0u;                               /* W 相 ADC 注入通道原始值。 */
static volatile uint16_t s_adc1_j1_raw = 0u;                         /* ADC1 注入 Rank1 原始值，用于采样链路诊断。 */
static volatile uint16_t s_adc1_j2_raw = 0u;                         /* ADC1 注入 Rank2 原始值，用于采样链路诊断。 */
static volatile uint16_t s_adc2_j1_raw = 0u;                         /* ADC2 注入 Rank1 原始值，用于采样链路诊断。 */
static volatile uint16_t s_adc2_j2_raw = 0u;                         /* ADC2 注入 Rank2 原始值，用于采样链路诊断。 */
static volatile uint16_t s_adc2_j3_raw = 0u;                         /* ADC2 注入 Rank3 原始值，用于采样链路诊断。 */
static volatile uint8_t s_svpwm_sector = 1u;                         /* 当前 SVPWM 电压矢量所在扇区，范围 1~6。 */
static volatile float s_svpwm_da = 0.5f;                             /* U 相占空比调试缓存。 */
static volatile float s_svpwm_db = 0.5f;                             /* V 相占空比调试缓存。 */
static volatile float s_svpwm_dc = 0.5f;                             /* W 相占空比调试缓存。 */
static volatile uint8_t s_midpoint_sampling = 1u;                    /* 当前是否可使用 PWM 中间点采样。 */
static volatile uint32_t s_offset_samples = 0u;                      /* 当前已经用于偏置平均的样本数。 */
static volatile float s_offset_sum_u = 0.0f;                         /* U 相偏置累加和。 */
static volatile float s_offset_sum_v = 0.0f;                         /* V 相偏置累加和。 */
static volatile float s_offset_sum_w = 0.0f;                         /* W 相偏置累加和。 */
static volatile uint32_t s_align_ticks = 0u;                         /* 对齐阶段已经运行的拍数。 */
static volatile float s_openloop_mech_freq_hz = FOC_OPENLOOP_START_FREQ_HZ; /* 开环机械频率。 */
static volatile float s_openloop_theta_e_rad = 0.0f;                 /* 当前开环电角度。 */
static volatile uint8_t s_adc1_ready = 0u;                           /* ADC1 是否已进入可用状态。 */
static volatile uint8_t s_is_running = 0u;                           /* 当前是否处于运行态。 */

/* observer 后台状态 */
static volatile float s_obs_theta_rad = 0.0f;                        /* observer 输出的电角度。 */
static volatile float s_obs_speed_rad_s = 0.0f;                      /* observer 输出的电角速度。 */
static volatile float s_obs_angle_err_deg = 0.0f;                    /* 开环角与观测角的误差，单位度。 */
static volatile uint8_t s_obs_locked = 0u;                           /* observer 是否被判定为锁定。 */
static volatile uint32_t s_obs_hold_ticks = 0u;                      /* 锁定条件连续满足的拍数。 */

/* donor FluxObserver_PLL 只作为后台计算内核保留。 */
static struct FluxObserver_PLL_t s_flux_obs = {0};

/* 开环角度 + dq 电流闭环调试状态。 */
static PIController_t s_id_pi = {0};
static PIController_t s_iq_pi = {0};
static volatile float s_id_ref_a = FOC_ID_REF_A;
static volatile float s_iq_ref_a = FOC_IQ_REF_A;
static volatile float s_id_meas_a = 0.0f;
static volatile float s_iq_meas_a = 0.0f;
static volatile float s_vd_cmd_v = 0.0f;
static volatile float s_vq_cmd_v = 0.0f;
static volatile float s_valpha_cmd_v = 0.0f;
static volatile float s_vbeta_cmd_v = 0.0f;
static volatile float s_theta_ctrl_rad = 0.0f;

static inline float foc_adc_to_voltage(uint16_t raw)
{
  return ((float)raw * FOC_ADC_VREF) / FOC_ADC_FULL_SCALE;
}

/* 把 ADC 电压转换成相电流。
 *
 * 公式含义：
 * - 先把 ADC 原始值换成电压；
 * - 再减去零点偏置；
 * - 最后除以“采样电阻 x 放大倍数”得到电流。
 *
 * `sign` 用来统一不同板子上的电流方向定义。
 * 如果电流方向和你的物理接线相反，后面看到的相电流也会反。
 */
static inline float foc_current_from_adc(float vadc, float offset_v, float sign)
{
  return sign * ((vadc - offset_v) / (FOC_GAIN * FOC_SHUNT_OHM));
}

static inline float foc_adc_to_bus_voltage(uint16_t raw)
{
  return foc_adc_to_voltage(raw) * FOC_VBUS_RATIO;
}

static inline void foc_cache_injected_raw_debug(uint16_t adc2_j1, uint16_t adc2_j2, uint16_t adc2_j3)
{
  s_adc1_j1_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  s_adc1_j2_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
  s_adc2_j1_raw = adc2_j1;
  s_adc2_j2_raw = adc2_j2;
  s_adc2_j3_raw = adc2_j3;
}

static uint8_t foc_voltage_sector(float valpha, float vbeta)
{
  const float two_pi = 6.2831853071795864769f;
  const float sector_width = 1.0471975511965977462f;
  float angle = atan2f(vbeta, valpha);
  if (angle < 0.0f)
  {
    angle += two_pi;
  }

  uint8_t sector = (uint8_t)(angle / sector_width) + 1u;
  if (sector > 6u)
  {
    sector = 6u;
  }
  return sector;
}

/* 把角度包到 [0, 2pi)。
 *
 * 角度在控制里通常只关心“当前相位”，不关心绝对累计值。
 * 所以每次积分后把角度折回这个区间，可以避免浮点数一直累加。
 */
static inline float foc_wrap_2pi(float angle)
{
  const float two_pi = 6.2831853071795864769f;
  while (angle >= two_pi) angle -= two_pi;
  while (angle < 0.0f) angle += two_pi;
  return angle;
}

/* 把角度包到 [-pi, pi]。
 *
 * 这个区间特别适合算“角差”：
 * - 正负号能直接告诉你偏前还是偏后；
 * - 数值不会在 0 和 2pi 之间跳来跳去；
 * - 串口或上位机画图时更容易看出误差是否收敛。
 */
static inline float foc_wrap_pi(float angle)
{
  const float pi = 3.14159265358979323846f;
  const float two_pi = 6.2831853071795864769f;
  while (angle > pi) angle -= two_pi;
  while (angle < -pi) angle += two_pi;
  return angle;
}

/* 调试打印时把弧度转成角度，便于在上位机里直观看误差。 */
static inline float foc_rad_to_deg(float rad)
{
  return rad * 57.29577951308232f;
}

/* 限制 PWM 比较值，避免写出边界以外的占空比。
 *
 * 在 SVPWM 里，比较值贴边通常意味着：
 * - 调制已经超出安全范围；
 * - 或者母线电压、参考电压有问题；
 * 所以这里保留少量余量。
 */
static inline uint16_t foc_clamp_ccr(int32_t ccr)
{
  if (ccr < (int32_t)FOC_SVPWM_MIN_CCR) return FOC_SVPWM_MIN_CCR;
  if (ccr > (int32_t)FOC_SVPWM_MAX_CCR) return FOC_SVPWM_MAX_CCR;
  return (uint16_t)ccr;
}

#if (FOC_OPENLOOP_CTRL_MODE == FOC_CTRL_MODE_OPENLOOP_CURRENT)
static void foc_limit_vector(float *x, float *y, float limit)
{
  if ((x == 0) || (y == 0) || (limit <= 0.0f))
  {
    return;
  }

  const float mag_sq = (*x * *x) + (*y * *y);
  const float limit_sq = limit * limit;
  if (mag_sq > limit_sq)
  {
    const float scale = limit / sqrtf(mag_sq);
    *x *= scale;
    *y *= scale;
  }
}
#endif

/* 把三相 PWM 都置于半周期中点。
 *
 * 中点输出等效于零矢量，适合做：
 * - 停机保持；
 * - 偏置校准；
 * - 故障后的安全收回。
 */
static void foc_set_tim1_mid(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, FOC_PWM_HALF_ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, FOC_PWM_HALF_ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, FOC_PWM_HALF_ARR);
}

/* 控制驱动器使能脚。
 *
 * 这里不做任何复杂状态判断，只做最直接的使能控制。
 * 驱动器是否真正上电，还要配合 PWM 和外部硬件条件一起看。
 */
static void foc_drive_enable(uint8_t enable)
{
  HAL_GPIO_WritePin(M1_EN_DRIVER_GPIO_Port, M1_EN_DRIVER_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 读取母线电压。
 *
 * 母线电压是 SVPWM 的归一化基准。
 * 如果它读错了，等效上就是“你以为输出了 4V，实际上可能输出了完全不同的电压比例”。
 */
static void foc_update_vbus(void)
{
  if (s_adc1_ready == 0u)
  {
    s_vbus_v = 24.0f;
    return;
  }

  uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
  s_vbus_v = foc_adc_to_bus_voltage(raw);
}

/* 读取三相电流。
 *
 * 当前通道按正点原子 DMG474 硬件定义：
 * - PC1 / ADC12_IN7 -> U 相；
 * - PC2 / ADC12_IN8 -> V 相；
 * - PC3 / ADC12_IN9 -> W 相。
 */
static void foc_update_currents(void)
{
  uint16_t raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
  uint16_t raw_v = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
  uint16_t raw_w = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
  foc_cache_injected_raw_debug(raw_u, raw_v, raw_w);
  s_raw_u = raw_u;
  s_raw_v = raw_v;
  s_raw_w = raw_w;
  float vu = foc_adc_to_voltage(raw_u);
  float vv = foc_adc_to_voltage(raw_v);
  float vw = foc_adc_to_voltage(raw_w);
  const float iu_adc = foc_current_from_adc(vu, s_iu_offset_v, FOC_CURR_SIGN_U);
  const float iv_adc = foc_current_from_adc(vv, s_iv_offset_v, FOC_CURR_SIGN_V);
  const float iw_adc = foc_current_from_adc(vw, s_iw_offset_v, FOC_CURR_SIGN_W);

  /* 三电阻低侧采样不能固定相信任三路 ADC。
   * 这里按 MCSDK R3_2 的扇区表选择两相，第三相用 Ia+Ib+Ic=0 重构：
   * - S1/S6: 采 V/W，重构 U；
   * - S2/S3: 采 U/W，重构 V；
   * - S4/S5: 采 U/V，重构 W。
   */
  switch (s_svpwm_sector)
  {
    case 1u:
    case 6u:
      s_iv_a = iv_adc;
      s_iw_a = iw_adc;
      s_iu_a = -(s_iv_a + s_iw_a);
      break;

    case 2u:
    case 3u:
      s_iu_a = iu_adc;
      s_iw_a = iw_adc;
      s_iv_a = -(s_iu_a + s_iw_a);
      break;

    case 4u:
    case 5u:
    default:
      s_iu_a = iu_adc;
      s_iv_a = iv_adc;
      s_iw_a = -(s_iu_a + s_iv_a);
      break;
  }
}

void FOC_SamplingUpdate(void)
{
  foc_update_vbus();
  foc_update_currents();
}

/* 检查运行中的基本故障条件。
 *
 * 这里的保护思路很朴素：
 * - 母线过低或过高，直接认为运行条件不成立；
 * - 电流过大，直接认为已经超出当前阶段的安全范围。
 *
 * 它不是完整的工业级保护，只是第一层“快刹车”。
 */
static uint8_t foc_fault_check(void)
{
  if (s_vbus_v < FOC_VBUS_UNDERVOLTAGE_V || s_vbus_v > FOC_VBUS_OVERVOLTAGE_V)
  {
    return 1u;
  }
  if (fabsf(s_iu_a) > FOC_CURRENT_LIMIT_A || fabsf(s_iv_a) > FOC_CURRENT_LIMIT_A || fabsf(s_iw_a) > FOC_CURRENT_LIMIT_A)
  {
    return 1u;
  }
  return 0u;
}

/* 把 alpha/beta 电压矢量换算成 TIM1 三相比较值。
 *
 * 计算过程：
 * 1. 先把 alpha/beta 投影成三相等效电压；
 * 2. 再找最大值和最小值；
 * 3. 用零序偏置把三相一起平移到 PWM 可用范围内。
 *
 * 这个过程的目的不是数学好看，而是让三相都尽量在可调范围内。
 */
static void foc_svpwm_to_tim1(float valpha, float vbeta)
{
  const float sqrt3 = 1.7320508075688772f;
  const float va = valpha;
  const float vb = (-0.5f * valpha) + (0.5f * sqrt3 * vbeta);
  const float vc = (-0.5f * valpha) - (0.5f * sqrt3 * vbeta);
  float vmax = va;
  float vmin = va;
  if (vb > vmax) vmax = vb;
  if (vc > vmax) vmax = vc;
  if (vb < vmin) vmin = vb;
  if (vc < vmin) vmin = vc;

  const float v_offset = 0.5f * (vmax + vmin);
  float da = 0.5f + (va - v_offset) / (s_vbus_v > 0.1f ? s_vbus_v : 24.0f);
  float db = 0.5f + (vb - v_offset) / (s_vbus_v > 0.1f ? s_vbus_v : 24.0f);
  float dc = 0.5f + (vc - v_offset) / (s_vbus_v > 0.1f ? s_vbus_v : 24.0f);

  if (da < 0.0f) da = 0.0f; if (da > 1.0f) da = 1.0f;
  if (db < 0.0f) db = 0.0f; if (db > 1.0f) db = 1.0f;
  if (dc < 0.0f) dc = 0.0f; if (dc > 1.0f) dc = 1.0f;

  s_svpwm_sector = foc_voltage_sector(valpha, vbeta);
  s_svpwm_da = da;
  s_svpwm_db = db;
  s_svpwm_dc = dc;

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, foc_clamp_ccr((int32_t)(da * (float)FOC_PWM_ARR)));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, foc_clamp_ccr((int32_t)(db * (float)FOC_PWM_ARR)));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, foc_clamp_ccr((int32_t)(dc * (float)FOC_PWM_ARR)));
}

static void foc_reset_current_loop(void)
{
  PIController_Init(&s_id_pi, FOC_ID_KP, FOC_ID_KI, -FOC_CURR_LOOP_V_LIMIT, FOC_CURR_LOOP_V_LIMIT);
  PIController_Init(&s_iq_pi, FOC_IQ_KP, FOC_IQ_KI, -FOC_CURR_LOOP_V_LIMIT, FOC_CURR_LOOP_V_LIMIT);
  PIController_Reset(&s_id_pi);
  PIController_Reset(&s_iq_pi);

  s_id_ref_a = FOC_ID_REF_A;
  s_iq_ref_a = FOC_IQ_REF_A;
  s_id_meas_a = 0.0f;
  s_iq_meas_a = 0.0f;
  s_vd_cmd_v = 0.0f;
  s_vq_cmd_v = 0.0f;
  s_valpha_cmd_v = 0.0f;
  s_vbeta_cmd_v = 0.0f;
  s_theta_ctrl_rad = 0.0f;
}

static void foc_update_dq_measurement(float theta_rad, ClarkePark_AlphaBeta_t *iab, ClarkePark_DQ_t *idq)
{
  ClarkePark_AlphaBeta_t local_ab = {0.0f, 0.0f};
  ClarkePark_DQ_t local_dq = {0.0f, 0.0f};

  ClarkePark_Clarke(s_iu_a, s_iv_a, s_iw_a, &local_ab);
  ClarkePark_Park(&local_ab, theta_rad, &local_dq);

  s_id_meas_a = local_dq.d;
  s_iq_meas_a = local_dq.q;
  s_theta_ctrl_rad = theta_rad;

  if (iab != 0) *iab = local_ab;
  if (idq != 0) *idq = local_dq;
}

#if (FOC_OPENLOOP_CTRL_MODE != FOC_CTRL_MODE_OPENLOOP_CURRENT)
static void foc_openloop_vf_command(float theta_rad, float *valpha, float *vbeta)
{
  ClarkePark_AlphaBeta_t iab = {0.0f, 0.0f};
  ClarkePark_DQ_t idq = {0.0f, 0.0f};
  foc_update_dq_measurement(theta_rad, &iab, &idq);

  (void)iab;
  s_id_ref_a = FOC_ID_REF_A;
  s_iq_ref_a = FOC_IQ_REF_A;
  s_vd_cmd_v = FOC_OPENLOOP_VDQ_V;
  s_vq_cmd_v = 0.0f;
  s_valpha_cmd_v = FOC_OPENLOOP_VDQ_V * cosf(theta_rad);
  s_vbeta_cmd_v = FOC_OPENLOOP_VDQ_V * sinf(theta_rad);

  if (valpha != 0) *valpha = s_valpha_cmd_v;
  if (vbeta != 0) *vbeta = s_vbeta_cmd_v;
}
#endif

#if (FOC_OPENLOOP_CTRL_MODE == FOC_CTRL_MODE_OPENLOOP_CURRENT)
static void foc_openloop_current_command(float theta_rad, float *valpha, float *vbeta)
{
  ClarkePark_AlphaBeta_t vab = {0.0f, 0.0f};
  ClarkePark_DQ_t idq = {0.0f, 0.0f};
  ClarkePark_DQ_t vdq = {0.0f, 0.0f};

  foc_update_dq_measurement(theta_rad, 0, &idq);

  s_id_ref_a = FOC_ID_REF_A;
  s_iq_ref_a = FOC_IQ_REF_A;
  vdq.d = PIController_Run(&s_id_pi, s_id_ref_a, idq.d, FOC_TS_SEC);
  vdq.q = PIController_Run(&s_iq_pi, s_iq_ref_a, idq.q, FOC_TS_SEC);
  foc_limit_vector(&vdq.d, &vdq.q, FOC_DQ_VOLT_LIMIT);

  ClarkePark_InvPark(&vdq, theta_rad, &vab);
  s_vd_cmd_v = vdq.d;
  s_vq_cmd_v = vdq.q;
  s_valpha_cmd_v = vab.alpha;
  s_vbeta_cmd_v = vab.beta;

  if (valpha != 0) *valpha = s_valpha_cmd_v;
  if (vbeta != 0) *vbeta = s_vbeta_cmd_v;
}
#endif

/* observer 更新。
 *
 * 这是调试里最关键的一段。
 * 它做的事情按顺序是：
 * - 把上一拍/当前拍的电压和电流送给观测器；
 * - 得到磁链矢量；
 * - 再把磁链矢量送入 PLL，提取角度和速度；
 * - 最后算开环角和观测角之间的误差。
 *
 * 如果这里不稳，后面的 `locked` 就没有意义。
 */
static void foc_observer_update(float valpha, float vbeta)
{
  /* 这一段是 observer 的“调试入口”。
   *
   * 它不直接参与控制闭环，而是做三件事：
   * 1. 把当前拍的电压/电流喂给磁链观测器；
   * 2. 读出观测器给出的角度和速度；
   * 3. 根据开环角和观测角的差值，判断 observer 是否已经稳定。
   *
   * 也就是说，这里更像一个“观测器监视器”，而不是控制器本身。
   */

  /* 1) 组织观测器输入。
   *
   * `valpha / vbeta` 是当前拍用于开环输出的电压矢量，
   * `s_iu_a / s_iv_a` 是当前拍测得的相电流。
   *
   * 观测器需要“电压 + 电流”一起看，才能估算出磁链。
   */
  s_flux_obs.Valpha_I = valpha; /* 当前拍 alpha 轴电压输入。 */
  s_flux_obs.Vbeta_I = vbeta;    /* 当前拍 beta 轴电压输入。 */
  s_flux_obs.Ialpha_I = s_iu_a;  /* 当前拍 alpha 轴电流输入。 */
  s_flux_obs.Ibeta_I = (s_iu_a + 2.0f * s_iv_a) * 0.57735026919f; /* Clarke 变换得到 beta 轴电流。 */

  /* 执行一次磁链观测器 + PLL 更新。
   *
   * 更新完成后，`s_flux_obs` 里会带回：
   * - 估算磁链矢量；
   * - 估算电角速度；
   * - 估算电角度。
   */
  FluxObserver_PLL_update(&s_flux_obs);

  /* 把观测器输出缓存到本模块，供串口打印和上位机曲线使用。 */
  s_obs_theta_rad = s_flux_obs.Etheta_O;      /* 观测到的电角度。 */
  s_obs_speed_rad_s = s_flux_obs.Espeed_O;    /* 观测到的电角速度。 */

  /* 2) 计算开环角与观测角之间的误差。
   *
   * 这里先把角差包到 [-pi, pi]，再转成角度。
   * 这样做的目的，是让误差在图上更容易看懂：
   * - 接近 0：说明跟得比较稳；
   * - 接近 +/-180 度：说明相位已经明显错位；
   * - 大幅跳变：通常意味着失锁或角度绕回。
   */
  const float angle_err_rad = foc_wrap_pi(s_openloop_theta_e_rad - s_obs_theta_rad);
  s_obs_angle_err_deg = foc_rad_to_deg(angle_err_rad);

  /* 3) 判断 observer 是否锁定。
   *
   * 锁定条件分两部分：
   * - 开环机械频率要足够高，太低时磁链方向本身就不稳定；
   * - 开环角和观测角的误差要足够小，并且要持续一段时间。
   *
   * 这里用“持续满足若干拍”而不是“一拍命中”，是为了避免偶发噪声把状态抖来抖去。
   */
  const float elec_freq_hz = s_openloop_mech_freq_hz * FOC_POLE_PAIRS; /* 机械频率换算成电频率。 */
  if (elec_freq_hz >= FOC_OBS_ENABLE_FREQ_HZ && fabsf(s_obs_angle_err_deg) < FOC_OBS_LOCK_ERR_DEG)
  {
    /* 当前拍满足锁定条件，就把连续满足计数往上加。 */
    if (s_obs_hold_ticks < (uint32_t)(FOC_OBS_LOCK_HOLD_MS / (FOC_TS_SEC * 1000.0f)))
    {
      s_obs_hold_ticks++;
    }
    else
    {
      /* 条件已经连续满足足够长时间，正式认为 observer 锁定。 */
      s_obs_locked = 1u;
    }
  }
  else
  {
    /* 只要条件中断，就清空累计计数并解除锁定。 */
    s_obs_hold_ticks = 0u;
    s_obs_locked = 0u;
  }
}

void FOC_BridgeInit(void)
{
  /* 清空所有运行态缓存，保证每次启动都从“干净状态”开始。
   * 尤其是 observer 的内部积分状态，如果残留上一次结果，会让第一次启动表现很怪。
   */
  s_state = FOC_STATE_IDLE;
  s_iu_offset_v = 0.0f;
  s_iv_offset_v = 0.0f;
  s_iw_offset_v = 0.0f;
  s_vbus_v = 24.0f;
  s_iu_a = s_iv_a = s_iw_a = 0.0f;
  s_raw_u = s_raw_v = s_raw_w = 0u;
  s_offset_samples = 0u;
  s_offset_sum_u = 0.0f;
  s_offset_sum_v = 0.0f;
  s_offset_sum_w = 0.0f;
  s_align_ticks = 0u;
  s_openloop_mech_freq_hz = FOC_OPENLOOP_START_FREQ_HZ;
  s_openloop_theta_e_rad = 0.0f;
  s_adc1_ready = 0u;
  s_is_running = 0u;
  s_obs_theta_rad = 0.0f;
  s_obs_speed_rad_s = 0.0f;
  s_obs_angle_err_deg = 0.0f;
  s_obs_locked = 0u;
  s_obs_hold_ticks = 0u;
  foc_reset_current_loop();
  foc_set_tim1_mid();
  foc_drive_enable(0u);
}

void FOC_Start(void)
{
  /* 启动 PWM、互补输出和 TIM1 基本定时器。
   *
   * 顺序上先把输出链路准备好，再让 ADC 进中断：
   * - 这样一旦采样开始，后级状态机就有完整的执行路径；
   * - 避免 ADC 先来、PWM 还没准备好的空窗期。
   */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_ADC_Start(&hadc1);
  s_adc1_ready = 1u;
  HAL_ADCEx_InjectedStart(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc2);
  s_is_running = 1u;

  /* 重新启动时清空 observer 内核，避免上一次运行残留状态影响本次结果。
   * 这一步很重要，因为磁链观测器和 PLL 都带积分属性。
   */
  FluxObserver_PLL_reset(&s_flux_obs);
  s_obs_theta_rad = 0.0f;
  s_obs_speed_rad_s = 0.0f;
  s_obs_angle_err_deg = 0.0f;
  s_obs_locked = 0u;
  s_obs_hold_ticks = 0u;
  foc_reset_current_loop();

  foc_set_tim1_mid();
  foc_drive_enable(0u);

  s_offset_samples = 0u;
  s_offset_sum_u = 0.0f;
  s_offset_sum_v = 0.0f;
  s_offset_sum_w = 0.0f;
  s_align_ticks = 0u;
  s_openloop_mech_freq_hz = FOC_OPENLOOP_START_FREQ_HZ;
  s_openloop_theta_e_rad = 0.0f;
  s_state = FOC_STATE_OFFSET_CALIB;
}

void FOC_Stop(void)
{
  /* 先把功率级拉回安全状态，再逐个关闭 ADC / TIM。
   * 这样停机时不会留下半使能状态。
   */
  FOC_FaultStop();
  HAL_ADCEx_InjectedStop(&hadc1);
  HAL_ADCEx_InjectedStop_IT(&hadc2);
  HAL_ADC_Stop(&hadc1);
  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  s_adc1_ready = 0u;
  s_is_running = 0u;
  s_state = FOC_STATE_IDLE;
}

FOC_StateTypeDef FOC_GetState(void)
{
  return s_state;
}

void FOC_CurrentOffsetCalib(void)
{
  /* 在零电流条件下采样若干次，估计传感器偏置。
   *
   * 这一步相当于给电流测量找“零点”。
   * 零点没找准，后面算出来的电流就会天然带偏置，observer 会把这个偏置当成真实物理量处理。
   */
  foc_set_tim1_mid();  /* 先把三相 PWM 拉到中点，避免对电流采样造成真实激励。 */
  foc_drive_enable(0u); /* 关闭驱动器，使电流采样尽量落在“无电流”条件下。 */

  uint16_t raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1); /* 读取 U 相注入转换原始值。 */
  uint16_t raw_v = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2); /* 读取 V 相注入转换原始值。 */
  uint16_t raw_w = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3); /* 读取 W 相注入转换原始值。 */
  foc_cache_injected_raw_debug(raw_u, raw_v, raw_w); /* 同步缓存 ADC1/ADC2 注入原始值，便于定位采样源。 */
  s_offset_sum_u += foc_adc_to_voltage(raw_u); /* 累加 U 相偏置对应的电压值。 */
  s_offset_sum_v += foc_adc_to_voltage(raw_v); /* 累加 V 相偏置对应的电压值。 */
  s_offset_sum_w += foc_adc_to_voltage(raw_w); /* 累加 W 相偏置对应的电压值。 */
  s_offset_samples++; /* 偏置样本计数加一。 */

  if (s_offset_samples >= FOC_CURRENT_OFFSET_CALIB_SAMPLES) /* 样本数够了，就计算平均偏置。 */
  {
    s_iu_offset_v = s_offset_sum_u / (float)s_offset_samples; /* U 相零点偏置电压。 */
    s_iv_offset_v = s_offset_sum_v / (float)s_offset_samples; /* V 相零点偏置电压。 */
    s_iw_offset_v = s_offset_sum_w / (float)s_offset_samples; /* W 相零点偏置电压。 */
    s_align_ticks = 0u; /* 对齐阶段重新从头计时。 */
    s_state = FOC_STATE_ALIGN; /* 偏置校准完成，进入对齐阶段。 */
  }
}

void FOC_AlignRun(void)
{
  /* 对齐阶段：
   * - 固定定子电角度；
   * - 输出小幅 d 轴电压；
   * - 让转子磁极先稳定到一个已知方向。
   *
   * 这一步的目标不是转起来，而是“先知道自己在哪”。
   */
  foc_set_tim1_mid();

  if (s_align_ticks == 0u)
  {
    foc_drive_enable(1u);
  }

  FOC_SamplingUpdate();
  if (foc_fault_check())
  {
    FOC_FaultStop();
    return;
  }

  s_openloop_theta_e_rad = 0.0f;
  foc_svpwm_to_tim1(FOC_ALIGN_VDQ_V, 0.0f);

  s_align_ticks++;
  if (s_align_ticks >= (uint32_t)(0.300f / FOC_TS_SEC))
  {
    s_openloop_mech_freq_hz = FOC_OPENLOOP_START_FREQ_HZ;
    s_openloop_theta_e_rad = 0.0f;
    foc_reset_current_loop();
    s_state = FOC_STATE_OPENLOOP;
  }
}

void FOC_OpenLoopRun(void)
{
  /* 开环起转：
   * - 频率从低速缓慢爬升到目标频率；
   * - 电压矢量按开环角度旋转；
   * - 每拍都把当前电压和电流送给 observer。
   *
   * 这一步是 observer 的“训练轨迹”。
   * 如果开环轨迹本身不稳定，observer 没有机会自己收敛出来。
   */
  FOC_SamplingUpdate(); /* 刷新母线电压和相电流，供保护和后续使用。 */
  if (foc_fault_check()) /* 若母线或电流已经越界，直接进入故障态。 */
  {
    FOC_FaultStop(); /* 把输出收回到安全状态。 */
    return; /* 故障时不继续执行后面的开环输出。 */
  }

  const float ramp_rate_hz_per_s = (FOC_OPENLOOP_TARGET_FREQ_HZ - FOC_OPENLOOP_START_FREQ_HZ) / (FOC_OPENLOOP_RAMP_TIME_MS * 0.001f); /* 计算每秒爬升多少机械频率。 */
  const float ramp_step_hz = ramp_rate_hz_per_s * FOC_TS_SEC; /* 把“每秒爬升量”换成“每拍爬升量”。 */
  
  if (s_openloop_mech_freq_hz < FOC_OPENLOOP_TARGET_FREQ_HZ) /* 如果还没爬到目标频率，就继续增加。 */
  {
    s_openloop_mech_freq_hz += ramp_step_hz; /* 当前拍把机械频率往上抬一点。 */
    if (s_openloop_mech_freq_hz > FOC_OPENLOOP_TARGET_FREQ_HZ) /* 最后一步不能超过目标值。 */
    {
      s_openloop_mech_freq_hz = FOC_OPENLOOP_TARGET_FREQ_HZ; /* 到顶后钳住。 */
    }
  }

  const float two_pi = 6.2831853071795864769f; /* 2pi 常量，用于把频率积分成角度。 */
  s_openloop_theta_e_rad += two_pi * (s_openloop_mech_freq_hz * FOC_POLE_PAIRS) * FOC_TS_SEC; /* 机械频率转成电角速度后做积分。 */
  s_openloop_theta_e_rad = foc_wrap_2pi(s_openloop_theta_e_rad); /* 把角度限制在 [0, 2pi) 内。 */

  float valpha = 0.0f;
  float vbeta = 0.0f;

#if (FOC_OPENLOOP_CTRL_MODE == FOC_CTRL_MODE_OPENLOOP_CURRENT)
  foc_openloop_current_command(s_openloop_theta_e_rad, &valpha, &vbeta); /* 开环角度 + dq 电流 PI。 */
#else
  foc_openloop_vf_command(s_openloop_theta_e_rad, &valpha, &vbeta); /* 原有开环电压输出路径。 */
#endif
  foc_svpwm_to_tim1(valpha, vbeta); /* 把 alpha/beta 电压换算成三相 PWM。 */

  foc_observer_update(valpha, vbeta); /* 用这一拍的电压和电流更新磁链观测器与 PLL。 */
}

void FOC_FaultStop(void)
{
  /* 故障态的唯一目标是把输出收回到安全点。
   * 这里不尝试自愈，避免把保护逻辑和调试逻辑混在一起。
   */
  foc_set_tim1_mid();
  foc_drive_enable(0u);
  s_state = FOC_STATE_FAULT;
}

void FOC_TaskHighFreq(void)
{
  /* 高速任务由 ADC 注入转换完成中断触发。
   *
   * 这是整个控制链最时间敏感的一层：
   * - 一次中断对应一次采样周期；
   * - 每次只推进状态机的一小步；
   * - 这样能确保采样、换算、更新、输出之间的时序一致。
   */
  switch (s_state)
  {
    case FOC_STATE_OFFSET_CALIB:
      FOC_CurrentOffsetCalib();
      break;
    case FOC_STATE_ALIGN:
      FOC_AlignRun();
      break;
    case FOC_STATE_OPENLOOP:
      FOC_OpenLoopRun();
      break;
    case FOC_STATE_FAULT:
      FOC_FaultStop();
      break;
    case FOC_STATE_IDLE:
    default:
      foc_set_tim1_mid();
      break;
  }
}

void FOC_TaskBackground(void)
{
  /* 当前版本后台没有额外任务。
   * 接口保留着，是为了以后加慢速统计、日志整理或低频监控时不用再改上层调用。
   */
}

float FOC_GetOpenLoopFreqHz(void) { return s_openloop_mech_freq_hz; }
float FOC_GetOpenLoopThetaErad(void) { return s_openloop_theta_e_rad; }
float FOC_GetFluxThetaRad(void)
{
  return atan2f(s_flux_obs.Flux_beta_O, s_flux_obs.Flux_alpha_O);
}
float FOC_GetObservedThetaRad(void) { return s_obs_theta_rad; }
float FOC_GetObservedSpeedRadPerSec(void) { return s_obs_speed_rad_s; }
float FOC_GetObservedAngleErrDeg(void) { return s_obs_angle_err_deg; }
uint8_t FOC_IsObserverLocked(void) { return s_obs_locked; }
float FOC_GetBusVoltageV(void) { return s_vbus_v; }
float FOC_GetPhaseCurrentUa(void) { return s_iu_a; }
float FOC_GetPhaseCurrentVa(void) { return s_iv_a; }
float FOC_GetPhaseCurrentWa(void) { return s_iw_a; }
void FOC_GetObserverInputs(float *valpha, float *vbeta, float *ialpha, float *ibeta, float *vbus)
{
  /* 对外返回与 observer 内核一致的 alpha/beta 输入，
   * 避免“调试口看到的 ibeta”和“observer 实际使用的 ibeta”不一致。
   */
  if (valpha) *valpha = s_flux_obs.Valpha_I;
  if (vbeta) *vbeta = s_flux_obs.Vbeta_I;
  if (ialpha) *ialpha = s_flux_obs.Ialpha_I;
  if (ibeta) *ibeta = s_flux_obs.Ibeta_I;
  if (vbus) *vbus = s_vbus_v;
}
void FOC_GetCurrentOffsets(float *iu_offset_v, float *iv_offset_v)
{
  if (iu_offset_v) *iu_offset_v = s_iu_offset_v;
  if (iv_offset_v) *iv_offset_v = s_iv_offset_v;
}

void FOC_GetCurrentOffsets3(float *iu_offset_v, float *iv_offset_v, float *iw_offset_v)
{
  if (iu_offset_v) *iu_offset_v = s_iu_offset_v;
  if (iv_offset_v) *iv_offset_v = s_iv_offset_v;
  if (iw_offset_v) *iw_offset_v = s_iw_offset_v;
}

void FOC_GetCurrentRawAdc(uint16_t *raw_u, uint16_t *raw_v, uint16_t *raw_w)
{
  if (raw_u) *raw_u = s_raw_u;
  if (raw_v) *raw_v = s_raw_v;
  if (raw_w) *raw_w = s_raw_w;
}

void FOC_GetInjectedRawAdcDebug(uint16_t *adc1_rank1,
                                uint16_t *adc1_rank2,
                                uint16_t *adc2_rank1,
                                uint16_t *adc2_rank2,
                                uint16_t *adc2_rank3)
{
  if (adc1_rank1) *adc1_rank1 = s_adc1_j1_raw;
  if (adc1_rank2) *adc1_rank2 = s_adc1_j2_raw;
  if (adc2_rank1) *adc2_rank1 = s_adc2_j1_raw;
  if (adc2_rank2) *adc2_rank2 = s_adc2_j2_raw;
  if (adc2_rank3) *adc2_rank3 = s_adc2_j3_raw;
}

void FOC_GetSvpwmDebug(uint8_t *sector, float *duty_u, float *duty_v, float *duty_w)
{
  if (sector) *sector = s_svpwm_sector;
  if (duty_u) *duty_u = s_svpwm_da;
  if (duty_v) *duty_v = s_svpwm_db;
  if (duty_w) *duty_w = s_svpwm_dc;
}

void FOC_GetCurrentLoopDebug(float *id_ref,
                             float *iq_ref,
                             float *id_meas,
                             float *iq_meas,
                             float *vd_cmd,
                             float *vq_cmd,
                             float *valpha_cmd,
                             float *vbeta_cmd,
                             float *theta_ctrl)
{
  if (id_ref) *id_ref = s_id_ref_a;
  if (iq_ref) *iq_ref = s_iq_ref_a;
  if (id_meas) *id_meas = s_id_meas_a;
  if (iq_meas) *iq_meas = s_iq_meas_a;
  if (vd_cmd) *vd_cmd = s_vd_cmd_v;
  if (vq_cmd) *vq_cmd = s_vq_cmd_v;
  if (valpha_cmd) *valpha_cmd = s_valpha_cmd_v;
  if (vbeta_cmd) *vbeta_cmd = s_vbeta_cmd_v;
  if (theta_ctrl) *theta_ctrl = s_theta_ctrl_rad;
}
