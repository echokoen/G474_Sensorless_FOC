#include "foc_pwm.h"
#include "bsp_tim.h"
#include "foc_config.h"
#include <math.h>

/* PWM / SVPWM 实现。
 *
 * 执行链路：
 * valpha/vbeta
 * -> foc_pwm_voltage_sector()
 * -> foc_pwm_get_sector_times()
 * -> FOC_PwmModule_ApplySvpwm()
 * -> TIM1 CH1/CH2/CH3 CCR
 * -> foc_pwm_update_adc_trigger_ccr()
 * -> TIM1 CH4 ADC 触发点
 *
 * 当前版本为显式扇区式 SVPWM，便于观察 sector、T1/T2/T0 和采样窗口。
 */

extern TIM_HandleTypeDef htim1;

static uint8_t foc_pwm_voltage_sector(float valpha, float vbeta)
{
  /* atan2 得到电压矢量角度，再按 60 度一扇区映射到 1~6。
   * 这里优先保证可读和正确性，后续如果 CPU 压力大可替换为无三角函数判区。
   */
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
  /* 三下桥电阻采样下，不同扇区可信的两相不同。
   * 这个映射必须和 foc_sampling_update_currents() 里的重构逻辑一致：
   * 1/6 -> VW，2/3 -> UW，4/5 -> UV。
   */
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
  /* 计算当前扇区内的有效矢量作用时间。
   *
   * theta_sector 是电压矢量在当前 60 度扇区内的局部角度；
   * T1/T2 对应该扇区两条相邻有效矢量；
   * T0 是剩余零矢量时间。
   */
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
    /* 简单过调制保护：如果 T1+T2 超过一个 PWM 周期，就等比例压回周期内。 */
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
  /* CCR 不允许贴到 0 或 ARR，给死区、采样和硬件边界留余量。 */
  if (ccr < (int32_t)FOC_SVPWM_MIN_CCR) return FOC_SVPWM_MIN_CCR;
  if (ccr > (int32_t)FOC_SVPWM_MAX_CCR) return FOC_SVPWM_MAX_CCR;
  return (uint16_t)ccr;
}

static uint16_t foc_pwm_max_u16(uint16_t a, uint16_t b)
{
  return (a >= b) ? a : b;
}

static void foc_pwm_select_trusted_pair_ccr(const FOC_PwmData_t *pwm,
                                            uint16_t *ccr_x,
                                            uint16_t *ccr_y)
{
  uint16_t local_x = pwm->ccr_u;
  uint16_t local_y = pwm->ccr_v;

  /*
   * 三下桥采样只使用当前扇区下的两相可信电流：
   *   sector 1/6 -> V/W
   *   sector 2/3 -> U/W
   *   sector 4/5 -> U/V
   * 这里返回这两相对应的 PWM CCR，用于判断低边公共导通窗口。
   */
  switch (pwm->svpwm_sector)
  {
    case 1u:
    case 6u:
      local_x = pwm->ccr_v;
      local_y = pwm->ccr_w;
      break;

    case 2u:
    case 3u:
      local_x = pwm->ccr_u;
      local_y = pwm->ccr_w;
      break;

    case 4u:
    case 5u:
    default:
      local_x = pwm->ccr_u;
      local_y = pwm->ccr_v;
      break;
  }

  if (ccr_x != 0) *ccr_x = local_x;
  if (ccr_y != 0) *ccr_y = local_y;
}

static void foc_pwm_update_adc_trigger_ccr(FOC_PwmData_t *pwm)
{
  /*
   * ADC 触发点优化策略（适配当前 TIM1 配置）：
   * - TIM1 CH1/2/3 是 PWM1，高边在 CNT < CCR 时导通；
   * - 低边互补输出在 CNT > CCR 后进入导通区；
   * - ADC 注入触发来自 TIM1_CH4 PWM2 的 OC4REF 上升沿，发生在上数 CNT == CCR4 附近；
   * - 因此，对于三下桥采样，采样点应放在“可信两相 CCR 均已越过之后”的公共低边窗口。
   *
   * 若当前可信两相 CCR 为 ccr_x/ccr_y，则安全窗口近似为：
   *     [max(ccr_x, ccr_y) + margin, FOC_PWM_ARR - margin]
   *
   * 这比固定采 2655 更安全：当某个可信相的 CCR 靠近或超过中点时，
   * 中点已经不在该相低边稳定导通窗口内，必须向后移动。
   */
  uint16_t ccr_x = pwm->ccr_u;
  uint16_t ccr_y = pwm->ccr_v;
  const uint16_t center = FOC_PWM_HALF_ARR;
  const uint16_t guard = FOC_ADC_TRIG_MARGIN_CCR;
  const uint16_t min_sample = FOC_ADC_SAMPLE_MIN_CCR;
  const uint16_t max_sample = FOC_ADC_SAMPLE_MAX_CCR;
  uint16_t pair_max;
  uint16_t safe_start;
  uint16_t safe_end;
  uint16_t safe_width;
  uint16_t sample;

  foc_pwm_select_trusted_pair_ccr(pwm, &ccr_x, &ccr_y);
  pair_max = foc_pwm_max_u16(ccr_x, ccr_y);

  if (pair_max > (uint16_t)(FOC_PWM_ARR - guard))
  {
    safe_start = FOC_PWM_ARR;
  }
  else
  {
    safe_start = (uint16_t)(pair_max + guard);
  }

  safe_end = max_sample;
  if (safe_start < min_sample)
  {
    safe_start = min_sample;
  }

  safe_width = (safe_start < safe_end) ? (uint16_t)(safe_end - safe_start) : 0u;

#if (FOC_ADC_DYNAMIC_SAMPLE_ENABLE != 0u)
  if (safe_width >= FOC_ADC_TRIG_MIN_WINDOW_CCR)
  {
    const uint16_t mid = (uint16_t)(safe_start + (safe_width / 2u));

    /*
     * 如果中点已经处在安全窗口内部，并且左右裕量都够，就继续用中点；
     * 否则移动到安全窗口中心。
     */
    if ((center >= safe_start) && (center <= safe_end) &&
        ((uint16_t)(center - safe_start) >= (FOC_ADC_TRIG_MIN_WINDOW_CCR / 2u)) &&
        ((uint16_t)(safe_end - center) >= (FOC_ADC_TRIG_MIN_WINDOW_CCR / 2u)))
    {
      sample = center;
    }
    else
    {
      sample = mid;
    }

    pwm->adc_window_ok = 1u;
    pwm->adc_trigger_fallback = 0u;
  }
  else
#endif
  {
    sample = FOC_ADC_TRIG_FALLBACK_CCR;
    pwm->adc_window_ok = 0u;
    pwm->adc_trigger_fallback = 1u;
  }

  sample = foc_pwm_clamp_ccr((int32_t)sample);

  pwm->adc_sample_ccr = sample;
  pwm->midpoint_sampling = (sample == center) ? 1u : 0u;
  pwm->adc_dynamic_sampling = (sample == center) ? 0u : 1u;

  /* win1/win2 表示当前采样点到安全窗口两侧的裕量。 */
  pwm->adc_win1_ccr = (sample > safe_start) ? (uint16_t)(sample - safe_start) : 0u;
  pwm->adc_win2_ccr = (sample < safe_end) ? (uint16_t)(safe_end - sample) : 0u;

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
  pwm->adc_window_ok = 0u;
  pwm->adc_dynamic_sampling = 0u;
}

void FOC_PwmModule_SetTim1Mid(FOC_PwmData_t *pwm)
{
  if (pwm == 0)
  {
    return;
  }

  /* 三相都给 50% duty，等效输出零电压矢量。 */
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
  pwm->adc_window_ok = 0u;
  pwm->adc_dynamic_sampling = 0u;
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

  /* 先判扇区，再计算当前扇区的 T1/T2/T0。 */
  sector = foc_pwm_voltage_sector(valpha, vbeta);
  foc_pwm_get_sector_times(valpha, vbeta, vbus_v, sector, &t1, &t2, &t0);
  half_t0 = 0.5f * t0;

  /* 中心对齐七段式等效分配。
   * ta/tb/tc 是每相上桥臂在一个周期内的等效导通时间。
   * 这里的分配表决定 sector SVPWM 的相序和 CCR 关系，调试时重点关注。
   */
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

  /* 导通时间归一化为 duty，再限幅到 [0,1]。 */
  da = ta / pwm_period;
  db = tb / pwm_period;
  dc = tc / pwm_period;

  if (da < 0.0f) da = 0.0f; if (da > 1.0f) da = 1.0f;
  if (db < 0.0f) db = 0.0f; if (db > 1.0f) db = 1.0f;
  if (dc < 0.0f) dc = 0.0f; if (dc > 1.0f) dc = 1.0f;

  /* 保存调试量并写入硬件 CCR。 */
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
