#include "foc_pwm.h"
#include "bsp_tim.h"
#include "foc_config.h"
#include <math.h>

/* PWM / SVPWM 实现。
 *
 * 负责扇区判断、T1/T2/T0 计算、CCR 更新以及采样触发点维护。
 */

extern TIM_HandleTypeDef htim1;

static uint8_t foc_pwm_voltage_sector(float valpha, float vbeta)
{
  const float two_pi = 6.2831853071795864769f;
  const float inv_sector_width = 0.95492965855f;
  float angle = atan2f(vbeta, valpha);

  if (angle < 0.0f)
  {
    angle += two_pi;
  }
  {
    uint8_t sector = (uint8_t)(angle * inv_sector_width) + 1u;
    if (sector > 6u)
    {
      sector = 6u;
    }
    return sector;
  }
}

static uint8_t foc_pwm_trusted_pair_from_sector(uint8_t sector)
{
  switch (sector)
  {
    case 1u:
    case 6u:
      return 1u;  /* VW */

    case 2u:
    case 3u:
      return 2u;  /* UW */

    case 4u:
    case 5u:
    default:
      return 3u;  /* UV */
  }
}

static void foc_pwm_get_sector_times(float valpha,
                                     float vbeta,
                                     float vbus_v,
                                     uint8_t sector,
                                     float *t1,
                                     float *t2,
                                     float *t0)
{
  const float two_pi = 6.2831853071795864769f;
  const float pi_over_3 = 1.0471975511965977462f;
  const float sqrt3 = 1.7320508075688772f;
  const float pwm_period = (float)FOC_PWM_ARR;
  const float norm_vbus = (vbus_v > 0.1f) ? vbus_v : 24.0f;
  float angle = atan2f(vbeta, valpha);
  float theta_sector;
  float vref;
  float local_t1;
  float local_t2;
  float local_t0;

  if (angle < 0.0f)
  {
    angle += two_pi;
  }

  theta_sector = angle - ((float)(sector - 1u) * pi_over_3);
  if (theta_sector < 0.0f)
  {
    theta_sector = 0.0f;
  }
  else if (theta_sector > pi_over_3)
  {
    theta_sector = pi_over_3;
  }

  vref = sqrtf((valpha * valpha) + (vbeta * vbeta));
  local_t1 = pwm_period * sqrt3 * (vref / norm_vbus) * sinf(pi_over_3 - theta_sector);
  local_t2 = pwm_period * sqrt3 * (vref / norm_vbus) * sinf(theta_sector);

  if (local_t1 < 0.0f) local_t1 = 0.0f;
  if (local_t2 < 0.0f) local_t2 = 0.0f;

  if ((local_t1 + local_t2) > pwm_period)
  {
    const float scale = pwm_period / (local_t1 + local_t2);
    local_t1 *= scale;
    local_t2 *= scale;
  }

  local_t0 = pwm_period - local_t1 - local_t2;
  if (local_t0 < 0.0f)
  {
    local_t0 = 0.0f;
  }

  if (t1) *t1 = local_t1;
  if (t2) *t2 = local_t2;
  if (t0) *t0 = local_t0;
}

static uint16_t foc_pwm_clamp_ccr(int32_t ccr)
{
  if (ccr < (int32_t)FOC_SVPWM_MIN_CCR) return FOC_SVPWM_MIN_CCR;
  if (ccr > (int32_t)FOC_SVPWM_MAX_CCR) return FOC_SVPWM_MAX_CCR;
  return (uint16_t)ccr;
}

static void foc_pwm_update_adc_trigger_ccr(FOC_PwmData_t *pwm)
{
  uint16_t ccr_x = pwm->ccr_u;
  uint16_t ccr_y = pwm->ccr_v;
  const uint16_t center = FOC_PWM_HALF_ARR;
  const int32_t sample_ccr = (int32_t)FOC_PWM_HALF_ARR + (int32_t)FOC_ADC_TRIG_TEST_OFFSET_CCR;

  switch (pwm->svpwm_sector)
  {
    case 1u:

    case 6u:

      ccr_x = pwm->ccr_v;
      ccr_y = pwm->ccr_w;
      break;

    case 2u:

    case 3u:

      ccr_x = pwm->ccr_u;
      ccr_y = pwm->ccr_w;
      break;

    case 4u:

    case 5u:

    default:

      ccr_x = pwm->ccr_u;
      ccr_y = pwm->ccr_v;
      break;
  }

  pwm->adc_win1_ccr = (ccr_x >= center) ? (uint16_t)(ccr_x - center) : (uint16_t)(center - ccr_x);
  pwm->adc_win2_ccr = (ccr_y >= center) ? (uint16_t)(ccr_y - center) : (uint16_t)(center - ccr_y);
  pwm->adc_sample_ccr = foc_pwm_clamp_ccr(sample_ccr);
  pwm->adc_trigger_fallback = 0u;
  pwm->midpoint_sampling = (FOC_ADC_TRIG_TEST_OFFSET_CCR == 0) ? 1u : 0u;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm->adc_sample_ccr);
}

void FOC_PwmModule_Init(FOC_PwmData_t *pwm)
{
  if (pwm == 0)
  {
    return;
  }

  pwm->svpwm_sector = 1u;
  pwm->svpwm_da = 0.5f;
  pwm->svpwm_db = 0.5f;
  pwm->svpwm_dc = 0.5f;
  pwm->svpwm_t1 = 0.0f;
  pwm->svpwm_t2 = 0.0f;
  pwm->svpwm_t0 = (float)FOC_PWM_ARR;
  pwm->ccr_u = FOC_PWM_HALF_ARR;
  pwm->ccr_v = FOC_PWM_HALF_ARR;
  pwm->ccr_w = FOC_PWM_HALF_ARR;
  pwm->adc_win1_ccr = 0u;
  pwm->adc_win2_ccr = 0u;
  pwm->adc_sample_ccr = FOC_ADC_TRIG_FALLBACK_CCR;
  pwm->adc_trigger_fallback = 1u;
  pwm->midpoint_sampling = 1u;
}

void FOC_PwmModule_SetTim1Mid(FOC_PwmData_t *pwm)
{
  if (pwm == 0)
  {
    return;
  }

  pwm->ccr_u = FOC_PWM_HALF_ARR;
  pwm->ccr_v = FOC_PWM_HALF_ARR;
  pwm->ccr_w = FOC_PWM_HALF_ARR;
  pwm->svpwm_da = 0.5f;
  pwm->svpwm_db = 0.5f;
  pwm->svpwm_dc = 0.5f;
  pwm->svpwm_t1 = 0.0f;
  pwm->svpwm_t2 = 0.0f;
  pwm->svpwm_t0 = (float)FOC_PWM_ARR;
  pwm->adc_win1_ccr = 0u;
  pwm->adc_win2_ccr = 0u;
  pwm->adc_sample_ccr = FOC_ADC_TRIG_FALLBACK_CCR;
  pwm->adc_trigger_fallback = 1u;
  pwm->midpoint_sampling = 1u;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm->ccr_u);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm->ccr_v);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm->ccr_w);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm->adc_sample_ccr);
}

void FOC_PwmModule_ApplySvpwm(FOC_PwmData_t *pwm, float valpha, float vbeta, float vbus_v)
{
  const float pwm_period = (float)FOC_PWM_ARR;
  uint8_t sector;
  float t1;
  float t2;
  float t0;
  float half_t0;
  float ta;
  float tb;
  float tc;
  float da;
  float db;
  float dc;

  if (pwm == 0)
  {
    return;
  }

  sector = foc_pwm_voltage_sector(valpha, vbeta);
  foc_pwm_get_sector_times(valpha, vbeta, vbus_v, sector, &t1, &t2, &t0);
  half_t0 = 0.5f * t0;

  /* 中心对齐七段式等效分配。 */
  switch (sector)
  {
    case 1u:
      ta = t1 + t2 + half_t0;
      tb = t2 + half_t0;
      tc = half_t0;
      break;

    case 2u:
      ta = t1 + half_t0;
      tb = t1 + t2 + half_t0;
      tc = half_t0;
      break;

    case 3u:
      ta = half_t0;
      tb = t1 + t2 + half_t0;
      tc = t2 + half_t0;
      break;

    case 4u:
      ta = half_t0;
      tb = t1 + half_t0;
      tc = t1 + t2 + half_t0;
      break;

    case 5u:
      ta = t2 + half_t0;
      tb = half_t0;
      tc = t1 + t2 + half_t0;
      break;

    case 6u:
    default:
      ta = t1 + t2 + half_t0;
      tb = half_t0;
      tc = t1 + half_t0;
      break;
  }

  da = ta / pwm_period;
  db = tb / pwm_period;
  dc = tc / pwm_period;

  if (da < 0.0f) da = 0.0f; if (da > 1.0f) da = 1.0f;
  if (db < 0.0f) db = 0.0f; if (db > 1.0f) db = 1.0f;
  if (dc < 0.0f) dc = 0.0f; if (dc > 1.0f) dc = 1.0f;

  pwm->svpwm_sector = sector;
  pwm->svpwm_da = da;
  pwm->svpwm_db = db;
  pwm->svpwm_dc = dc;
  pwm->svpwm_t1 = t1;
  pwm->svpwm_t2 = t2;
  pwm->svpwm_t0 = t0;
  pwm->ccr_u = foc_pwm_clamp_ccr((int32_t)(da * (float)FOC_PWM_ARR));
  pwm->ccr_v = foc_pwm_clamp_ccr((int32_t)(db * (float)FOC_PWM_ARR));
  pwm->ccr_w = foc_pwm_clamp_ccr((int32_t)(dc * (float)FOC_PWM_ARR));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm->ccr_u);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm->ccr_v);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm->ccr_w);
  foc_pwm_update_adc_trigger_ccr(pwm);
}

void FOC_PwmModule_GetSvpwmDebug(const FOC_PwmData_t *pwm, uint8_t *sector, float *duty_u, float *duty_v, float *duty_w)
{
  if (pwm == 0)
  {
    return;
  }

  if (sector) *sector = pwm->svpwm_sector;
  if (duty_u) *duty_u = pwm->svpwm_da;
  if (duty_v) *duty_v = pwm->svpwm_db;
  if (duty_w) *duty_w = pwm->svpwm_dc;
}

void FOC_PwmModule_GetSamplingWindowDebug(const FOC_PwmData_t *pwm,
                                   uint8_t *sector,
                                   uint16_t *ccr_u,
                                   uint16_t *ccr_v,
                                   uint16_t *ccr_w,
                                   uint16_t *win1,
                                   uint16_t *win2,
                                   uint16_t *sample_ccr,
                                   uint8_t *used_fallback,
                                   uint8_t *trusted_pair)
{
  if (pwm == 0)
  {
    return;
  }

  if (sector) *sector = pwm->svpwm_sector;
  if (ccr_u) *ccr_u = pwm->ccr_u;
  if (ccr_v) *ccr_v = pwm->ccr_v;
  if (ccr_w) *ccr_w = pwm->ccr_w;
  if (win1) *win1 = pwm->adc_win1_ccr;
  if (win2) *win2 = pwm->adc_win2_ccr;
  if (sample_ccr) *sample_ccr = pwm->adc_sample_ccr;
  if (used_fallback) *used_fallback = pwm->adc_trigger_fallback;
  if (trusted_pair) *trusted_pair = foc_pwm_trusted_pair_from_sector(pwm->svpwm_sector);
}
