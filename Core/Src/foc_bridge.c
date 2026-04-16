#include "foc_bridge.h"
#include <math.h>

/* 第一阶段只做电压开环。
 * 为什么这样做：先把启动链、采样链、保护链打通，
 * 不把电流 PI 和 observer 放进主控制环，降低上板试转风险。 */

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

static volatile FOC_StateTypeDef s_state = FOC_STATE_IDLE;
static volatile float s_iu_offset_v = 0.0f;
static volatile float s_iv_offset_v = 0.0f;
static volatile float s_vbus_v = 0.0f;
static volatile float s_iu_a = 0.0f;
static volatile float s_iv_a = 0.0f;
static volatile float s_iw_a = 0.0f;
static volatile uint32_t s_offset_samples = 0u;
static volatile float s_offset_sum_u = 0.0f;
static volatile float s_offset_sum_v = 0.0f;
static volatile uint32_t s_align_ticks = 0u;
static volatile float s_openloop_mech_freq_hz = FOC_OPENLOOP_START_FREQ_HZ;
static volatile float s_openloop_theta_e_rad = 0.0f;
static volatile uint8_t s_adc1_ready = 0u;
static volatile uint8_t s_is_running = 0u;

static inline float foc_adc_to_voltage(uint16_t raw)
{
  return ((float)raw * FOC_ADC_VREF) / FOC_ADC_FULL_SCALE;
}

static inline float foc_current_from_adc(float vadc, float offset_v, float sign)
{
  return sign * ((vadc - offset_v) / (FOC_GAIN * FOC_SHUNT_OHM));
}

static inline float foc_wrap_2pi(float angle)
{
  const float two_pi = 6.2831853071795864769f;
  while (angle >= two_pi) angle -= two_pi;
  while (angle < 0.0f) angle += two_pi;
  return angle;
}

static inline uint16_t foc_clamp_ccr(int32_t ccr)
{
  if (ccr < (int32_t)FOC_SVPWM_MIN_CCR) return FOC_SVPWM_MIN_CCR;
  if (ccr > (int32_t)FOC_SVPWM_MAX_CCR) return FOC_SVPWM_MAX_CCR;
  return (uint16_t)ccr;
}

static void foc_set_tim1_mid(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, FOC_PWM_HALF_ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, FOC_PWM_HALF_ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, FOC_PWM_HALF_ARR);
}

static void foc_drive_enable(uint8_t enable)
{
  HAL_GPIO_WritePin(M1_EN_DRIVER_GPIO_Port, M1_EN_DRIVER_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void foc_update_vbus(void)
{
  if (s_adc1_ready == 0u)
  {
    s_vbus_v = 24.0f;
    return;
  }

  /* 方案A：ADC1 连续转换，控制周期内直接读取最新规则组结果。 */
  uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
  s_vbus_v = foc_adc_to_voltage(raw) * FOC_VBUS_RATIO;
}

static void foc_update_currents(void)
{
  uint16_t raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
  uint16_t raw_v = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
  float vu = foc_adc_to_voltage(raw_u);
  float vv = foc_adc_to_voltage(raw_v);
  s_iu_a = foc_current_from_adc(vu, s_iu_offset_v, FOC_CURR_SIGN_U);
  s_iv_a = foc_current_from_adc(vv, s_iv_offset_v, FOC_CURR_SIGN_V);
  s_iw_a = -(s_iu_a + s_iv_a);
}

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

static void foc_svpwm_to_tim1(float valpha, float vbeta)
{
  /* 这里保留最小 SVPWM 数学映射，后续阶段可替换为 donor 版本。 */
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

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, foc_clamp_ccr((int32_t)(da * (float)FOC_PWM_ARR)));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, foc_clamp_ccr((int32_t)(db * (float)FOC_PWM_ARR)));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, foc_clamp_ccr((int32_t)(dc * (float)FOC_PWM_ARR)));
}

void FOC_BridgeInit(void)
{
  s_state = FOC_STATE_IDLE;
  s_iu_offset_v = 0.0f;
  s_iv_offset_v = 0.0f;
  s_vbus_v = 24.0f;
  s_iu_a = s_iv_a = s_iw_a = 0.0f;
  s_offset_samples = 0u;
  s_offset_sum_u = 0.0f;
  s_offset_sum_v = 0.0f;
  s_align_ticks = 0u;
  s_openloop_mech_freq_hz = FOC_OPENLOOP_START_FREQ_HZ;
  s_openloop_theta_e_rad = 0.0f;
  s_adc1_ready = 0u;
  foc_set_tim1_mid();
  foc_drive_enable(0u);
}

void FOC_Start(void)
{
  /* 启动链：先启动 PWM/互补输出，再启动 ADC，再进入 offset 校准。 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim1);

  /* ADC1 采用连续转换，避免每次读母线电压都重新启动。
   * 这样更稳定，也更适合第一阶段开环试转。 */
  HAL_ADC_Start(&hadc1);
  s_adc1_ready = 1u;
  HAL_ADCEx_InjectedStart_IT(&hadc2);
  s_is_running = 1u;

  foc_set_tim1_mid();
  foc_drive_enable(0u);

  s_offset_samples = 0u;
  s_offset_sum_u = 0.0f;
  s_offset_sum_v = 0.0f;
  s_align_ticks = 0u;
  s_openloop_mech_freq_hz = FOC_OPENLOOP_START_FREQ_HZ;
  s_openloop_theta_e_rad = 0.0f;
  s_state = FOC_STATE_OFFSET_CALIB;
}

void FOC_Stop(void)
{
  /* 停止链路：先切保护，再停 ADC/TIM，避免输出和采样继续跑。 */
  FOC_FaultStop();
  HAL_ADCEx_InjectedStop_IT(&hadc2);
  HAL_ADC_Stop(&hadc1);
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
  foc_set_tim1_mid();
  foc_drive_enable(0u);

  uint16_t raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
  uint16_t raw_v = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
  s_offset_sum_u += foc_adc_to_voltage(raw_u);
  s_offset_sum_v += foc_adc_to_voltage(raw_v);
  s_offset_samples++;

  if (s_offset_samples >= FOC_CURRENT_OFFSET_CALIB_SAMPLES)
  {
    s_iu_offset_v = s_offset_sum_u / (float)s_offset_samples;
    s_iv_offset_v = s_offset_sum_v / (float)s_offset_samples;
    s_align_ticks = 0u;
    s_state = FOC_STATE_ALIGN;
  }
}

void FOC_AlignRun(void)
{
  foc_set_tim1_mid();

  /* 对齐阶段保持 300ms 左右，避免一个周期就切换到 OPENLOOP。 */
  if (s_align_ticks == 0u)
  {
    foc_drive_enable(1u);
  }

  foc_update_vbus();
  foc_update_currents();
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
    s_state = FOC_STATE_OPENLOOP;
  }
}

void FOC_OpenLoopRun(void)
{
  foc_update_vbus();
  foc_update_currents();
  if (foc_fault_check())
  {
    FOC_FaultStop();
    return;
  }

  /* 这里的开环频率定义为机械频率（Hz）。
   * 电角频率 = 机械频率 × 极对数。
   * 斜率严格使用 FOC_OPENLOOP_RAMP_TIME_MS。 */
  const float ramp_rate_hz_per_s = (FOC_OPENLOOP_TARGET_FREQ_HZ - FOC_OPENLOOP_START_FREQ_HZ) / (FOC_OPENLOOP_RAMP_TIME_MS * 0.001f);
  const float ramp_step_hz = ramp_rate_hz_per_s * FOC_TS_SEC;
  if (s_openloop_mech_freq_hz < FOC_OPENLOOP_TARGET_FREQ_HZ)
  {
    s_openloop_mech_freq_hz += ramp_step_hz;
    if (s_openloop_mech_freq_hz > FOC_OPENLOOP_TARGET_FREQ_HZ)
    {
      s_openloop_mech_freq_hz = FOC_OPENLOOP_TARGET_FREQ_HZ;
    }
  }

  const float two_pi = 6.2831853071795864769f;
  s_openloop_theta_e_rad += two_pi * (s_openloop_mech_freq_hz * FOC_POLE_PAIRS) * FOC_TS_SEC;
  s_openloop_theta_e_rad = foc_wrap_2pi(s_openloop_theta_e_rad);

  /* 第一阶段电压开环：直接用固定 Vd/Vq 做开环旋转，不把 PI 电流环接入主链。 */
  const float valpha = FOC_OPENLOOP_VDQ_V * cosf(s_openloop_theta_e_rad);
  const float vbeta = FOC_OPENLOOP_VDQ_V * sinf(s_openloop_theta_e_rad);
  foc_svpwm_to_tim1(valpha, vbeta);
}

void FOC_FaultStop(void)
{
  foc_set_tim1_mid();
  foc_drive_enable(0u);
  s_state = FOC_STATE_FAULT;
}

void FOC_TaskHighFreq(void)
{
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
  /* 第二阶段再接 observer / EKF。第一阶段后台仅预留接口。 */
}
