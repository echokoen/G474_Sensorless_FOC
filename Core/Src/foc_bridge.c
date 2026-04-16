#include "foc_bridge.h"
#include "PMSM_Control_Core/FluxObserver_PLL.h"
#include <math.h>

/* 第二阶段：observer 后台运行，但不接管主控制角。
 * 为什么这样做：先保持开环控制不变，只让观测器跟踪，
 * 便于验证角度和速度估算是否收敛。 */

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

/* observer 后台状态 */
static volatile float s_obs_theta_rad = 0.0f;
static volatile float s_obs_speed_rad_s = 0.0f;
static volatile float s_obs_angle_err_deg = 0.0f;
static volatile uint8_t s_obs_locked = 0u;
static volatile uint32_t s_obs_hold_ticks = 0u;

/* donor FluxObserver_PLL 只作为后台计算内核保留。 */
static struct FluxObserver_PLL_t s_flux_obs = {0};

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

static inline float foc_wrap_pi(float angle)
{
  const float pi = 3.14159265358979323846f;
  const float two_pi = 6.2831853071795864769f;
  while (angle > pi) angle -= two_pi;
  while (angle < -pi) angle += two_pi;
  return angle;
}

static inline float foc_rad_to_deg(float rad)
{
  return rad * 57.29577951308232f;
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

static void foc_observer_update(float valpha, float vbeta)
{
  /* donor 磁链观测器后台运行，不接管控制，只用于验证跟踪稳定性。 */
  s_flux_obs.Valpha_I = valpha;
  s_flux_obs.Vbeta_I = vbeta;
  s_flux_obs.Ialpha_I = s_iu_a;
  s_flux_obs.Ibeta_I = (s_iu_a + 2.0f * s_iv_a) * 0.57735026919f;
  FluxObserver_PLL_update(&s_flux_obs);

  s_obs_theta_rad = s_flux_obs.Etheta_O;
  s_obs_speed_rad_s = s_flux_obs.Espeed_O;

  /* 角差先包到 [-pi, pi]，再转成角度。 */
  const float angle_err_rad = foc_wrap_pi(s_openloop_theta_e_rad - s_obs_theta_rad);
  s_obs_angle_err_deg = foc_rad_to_deg(angle_err_rad);

  /* lock 判定按电角频率，而不是机械频率。 */
  const float elec_freq_hz = s_openloop_mech_freq_hz * FOC_POLE_PAIRS;
  if (elec_freq_hz >= FOC_OBS_ENABLE_FREQ_HZ && fabsf(s_obs_angle_err_deg) < FOC_OBS_LOCK_ERR_DEG)
  {
    if (s_obs_hold_ticks < (uint32_t)(FOC_OBS_LOCK_HOLD_MS / (FOC_TS_SEC * 1000.0f)))
    {
      s_obs_hold_ticks++;
    }
    else
    {
      s_obs_locked = 1u;
    }
  }
  else
  {
    s_obs_hold_ticks = 0u;
    s_obs_locked = 0u;
  }
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
  s_is_running = 0u;
  s_obs_theta_rad = 0.0f;
  s_obs_speed_rad_s = 0.0f;
  s_obs_angle_err_deg = 0.0f;
  s_obs_locked = 0u;
  s_obs_hold_ticks = 0u;
  foc_set_tim1_mid();
  foc_drive_enable(0u);
}

void FOC_Start(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_ADC_Start(&hadc1);
  s_adc1_ready = 1u;
  HAL_ADCEx_InjectedStart_IT(&hadc2);
  s_is_running = 1u;

  /* 重新启动时清空 observer 内核，避免残留状态影响第二阶段稳定性。 */
  FluxObserver_PLL_reset(&s_flux_obs);
  s_obs_theta_rad = 0.0f;
  s_obs_speed_rad_s = 0.0f;
  s_obs_angle_err_deg = 0.0f;
  s_obs_locked = 0u;
  s_obs_hold_ticks = 0u;

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
  FOC_FaultStop();
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

  const float valpha = FOC_OPENLOOP_VDQ_V * cosf(s_openloop_theta_e_rad);
  const float vbeta = FOC_OPENLOOP_VDQ_V * sinf(s_openloop_theta_e_rad);
  foc_svpwm_to_tim1(valpha, vbeta);

  foc_observer_update(valpha, vbeta);
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
  /* 第二阶段后台仅保留接口。 */
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
