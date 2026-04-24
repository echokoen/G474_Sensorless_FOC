#include "foc_sampling.h"
#include "bsp_adc.h"
#include "foc_config.h"
#include "foc_math.h"

/* 采样聚合实现。
 *
 * 负责读取 ADC 原始值、换算相电流和母线电压，并维护零点校准。
 */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

static float foc_sampling_adc_to_voltage(uint16_t raw)
{
  /* ADC 原始计数 -> 引脚电压。 */
  const float adc_lsb_v = FOC_ADC_VREF / FOC_ADC_FULL_SCALE;
  return (float)raw * adc_lsb_v;
}

static float foc_sampling_current_from_adc(float vadc, float offset_v, float sign)
{
  /* 采样电压 -> 相电流：
   * I = (Vadc - Voffset) / (放大倍数 * 采样电阻)
   * sign 用于适配硬件放大器方向或相线定义。
   */
  const float inv_gain_shunt = 1.0f / (FOC_GAIN * FOC_SHUNT_OHM);
  return sign * (vadc - offset_v) * inv_gain_shunt;
}

static float foc_sampling_adc_to_bus_voltage(uint16_t raw)
{
  return foc_sampling_adc_to_voltage(raw) * FOC_VBUS_RATIO;
}

static void foc_sampling_cache_injected_raw_debug(FOC_SamplingData_t *sampling,
                                                  uint16_t adc2_j1,
                                                  uint16_t adc2_j2,
                                                  uint16_t adc2_j3)
{
  sampling->adc1_j1_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  sampling->adc1_j2_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
  sampling->adc2_j1_raw = adc2_j1;
  sampling->adc2_j2_raw = adc2_j2;
  sampling->adc2_j3_raw = adc2_j3;
}

static void foc_sampling_update_vbus(FOC_SamplingData_t *sampling)
{
  if (sampling->adc1_ready == 0u)
  {
    sampling->vbus_v = 24.0f;
    return;
  }

  sampling->vbus_v = foc_sampling_adc_to_bus_voltage((uint16_t)HAL_ADC_GetValue(&hadc1));
}

static void foc_sampling_update_currents(FOC_SamplingData_t *sampling)
{
  /* 读取 ADC2 注入 rank。
   * 当前约定：
   * rank1 -> U，rank2 -> V，rank3 -> W。
   * 如果 CubeMX 通道顺序变化，这里必须同步更新。
   */
  const uint16_t raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
  const uint16_t raw_v = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
  const uint16_t raw_w = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
  const float vu = foc_sampling_adc_to_voltage(raw_u);
  const float vv = foc_sampling_adc_to_voltage(raw_v);
  const float vw = foc_sampling_adc_to_voltage(raw_w);
  const float iu_adc = foc_sampling_current_from_adc(vu, sampling->iu_offset_v, FOC_CURR_SIGN_U);
  const float iv_adc = foc_sampling_current_from_adc(vv, sampling->iv_offset_v, FOC_CURR_SIGN_V);
  const float iw_adc = foc_sampling_current_from_adc(vw, sampling->iw_offset_v, FOC_CURR_SIGN_W);
  float iu_new = 0.0f;
  float iv_new = 0.0f;
  float iw_new = 0.0f;

  foc_sampling_cache_injected_raw_debug(sampling, raw_u, raw_v, raw_w);
  sampling->raw_u = raw_u;
  sampling->raw_v = raw_v;
  sampling->raw_w = raw_w;

  switch (sampling->sample_sector)
  {
    case 1u:
    case 6u:
      /* 扇区 1/6：V/W 两相更可信，U 相用三相和为 0 重构。 */
      sampling->trusted_current_pair = 1u;
      iv_new = iv_adc;
      iw_new = iw_adc;
      iu_new = -(iv_new + iw_new);
      break;

    case 2u:
    case 3u:
      /* 扇区 2/3：U/W 两相更可信，V 相重构。 */
      sampling->trusted_current_pair = 2u;
      iu_new = iu_adc;
      iw_new = iw_adc;
      iv_new = -(iu_new + iw_new);
      break;

    case 4u:
    case 5u:
    default:
      /* 扇区 4/5：U/V 两相更可信，W 相重构。 */
      sampling->trusted_current_pair = 3u;
      iu_new = iu_adc;
      iv_new = iv_adc;
      iw_new = -(iu_new + iv_new);
      break;
  }

#if (FOC_ADC_HOLD_LAST_ON_FALLBACK != 0u)
  if ((sampling->fallback_request != 0u) && (sampling->last_valid_current_ready != 0u))
  {
    sampling->iu_a = sampling->last_valid_iu_a;
    sampling->iv_a = sampling->last_valid_iv_a;
    sampling->iw_a = sampling->last_valid_iw_a;
    sampling->used_hold_last_current = 1u;
    return;
  }
#endif

  sampling->iu_a = iu_new;
  sampling->iv_a = iv_new;
  sampling->iw_a = iw_new;
  sampling->last_valid_iu_a = iu_new;
  sampling->last_valid_iv_a = iv_new;
  sampling->last_valid_iw_a = iw_new;
  sampling->last_valid_current_ready = 1u;
  sampling->used_hold_last_current = 0u;
}

void FOC_SamplingModule_Init(FOC_SamplingData_t *sampling)
{
  if (sampling == 0)
  {
    return;
  }

  sampling->iu_offset_v = 0.0f;
  sampling->iv_offset_v = 0.0f;
  sampling->iw_offset_v = 0.0f;
  sampling->vbus_v = 24.0f;
  sampling->iu_a = 0.0f;
  sampling->iv_a = 0.0f;
  sampling->iw_a = 0.0f;
  sampling->last_valid_iu_a = 0.0f;
  sampling->last_valid_iv_a = 0.0f;
  sampling->last_valid_iw_a = 0.0f;
  sampling->last_valid_current_ready = 0u;
  sampling->fallback_request = 0u;
  sampling->used_hold_last_current = 0u;
  sampling->raw_u = 0u;
  sampling->raw_v = 0u;
  sampling->raw_w = 0u;
  sampling->adc1_j1_raw = 0u;
  sampling->adc1_j2_raw = 0u;
  sampling->adc2_j1_raw = 0u;
  sampling->adc2_j2_raw = 0u;
  sampling->adc2_j3_raw = 0u;
  sampling->sample_sector = 1u;
  sampling->trusted_current_pair = 0u;
  sampling->offset_samples = 0u;
  sampling->offset_sum_u = 0.0f;
  sampling->offset_sum_v = 0.0f;
  sampling->offset_sum_w = 0.0f;
  sampling->adc1_ready = 0u;
}

void FOC_SamplingModule_SetAdc1Ready(FOC_SamplingData_t *sampling, uint8_t ready)
{
  if (sampling != 0)
  {
    sampling->adc1_ready = ready;
  }
}

void FOC_SamplingModule_SetSampleSector(FOC_SamplingData_t *sampling, uint8_t sector)
{
  if (sampling != 0)
  {
    sampling->sample_sector = sector;
  }
}

void FOC_SamplingModule_SetFallbackRequest(FOC_SamplingData_t *sampling, uint8_t fallback)
{
  if (sampling != 0)
  {
    sampling->fallback_request = fallback;
  }
}

void FOC_SamplingModule_Update(FOC_SamplingData_t *sampling)
{
  if (sampling == 0)
  {
    return;
  }

  foc_sampling_update_vbus(sampling);
  foc_sampling_update_currents(sampling);
}

void FOC_SamplingModule_RunOffsetCalib(FOC_SamplingData_t *sampling, FOC_StateTypeDef *state)
{
  const uint16_t raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
  const uint16_t raw_v = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
  const uint16_t raw_w = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);

  if (sampling == 0)
  {
    return;
  }

  /* offset 校准要求电机无有效电流输出。
   * 这里只累计 ADC 电压平均值，作为后续电流换算零点。
   */
  foc_sampling_cache_injected_raw_debug(sampling, raw_u, raw_v, raw_w);
  sampling->offset_sum_u += foc_sampling_adc_to_voltage(raw_u);
  sampling->offset_sum_v += foc_sampling_adc_to_voltage(raw_v);
  sampling->offset_sum_w += foc_sampling_adc_to_voltage(raw_w);
  sampling->offset_samples++;

  if (sampling->offset_samples >= FOC_CURRENT_OFFSET_CALIB_SAMPLES)
  {
    const float inv_offset_samples = 1.0f / (float)sampling->offset_samples;
    sampling->iu_offset_v = sampling->offset_sum_u * inv_offset_samples;
    sampling->iv_offset_v = sampling->offset_sum_v * inv_offset_samples;
    sampling->iw_offset_v = sampling->offset_sum_w * inv_offset_samples;

    if (state != 0)
    {
      *state = FOC_STATE_ALIGN;
    }
  }
}

float FOC_SamplingModule_GetBusVoltageV(const FOC_SamplingData_t *sampling)
{
  return (sampling != 0) ? sampling->vbus_v : 0.0f;
}

float FOC_SamplingModule_GetPhaseCurrentUa(const FOC_SamplingData_t *sampling)
{
  return (sampling != 0) ? sampling->iu_a : 0.0f;
}

float FOC_SamplingModule_GetPhaseCurrentVa(const FOC_SamplingData_t *sampling)
{
  return (sampling != 0) ? sampling->iv_a : 0.0f;
}

float FOC_SamplingModule_GetPhaseCurrentWa(const FOC_SamplingData_t *sampling)
{
  return (sampling != 0) ? sampling->iw_a : 0.0f;
}

void FOC_SamplingModule_GetCurrentOffsets(const FOC_SamplingData_t *sampling, float *iu_offset_v, float *iv_offset_v, float *iw_offset_v)
{
  if (sampling == 0)
  {
    return;
  }

  if (iu_offset_v) *iu_offset_v = sampling->iu_offset_v;
  if (iv_offset_v) *iv_offset_v = sampling->iv_offset_v;
  if (iw_offset_v) *iw_offset_v = sampling->iw_offset_v;
}

void FOC_SamplingModule_GetCurrentRawAdc(const FOC_SamplingData_t *sampling, uint16_t *raw_u, uint16_t *raw_v, uint16_t *raw_w)
{
  if (sampling == 0)
  {
    return;
  }

  if (raw_u) *raw_u = sampling->raw_u;
  if (raw_v) *raw_v = sampling->raw_v;
  if (raw_w) *raw_w = sampling->raw_w;
}

void FOC_SamplingModule_GetInjectedRawAdcDebug(const FOC_SamplingData_t *sampling,
                                                uint16_t *adc1_rank1,
                                                uint16_t *adc1_rank2,
                                                uint16_t *adc2_rank1,
                                                uint16_t *adc2_rank2,
                                                uint16_t *adc2_rank3)
{
  if (sampling == 0)
  {
    return;
  }

  if (adc1_rank1) *adc1_rank1 = sampling->adc1_j1_raw;
  if (adc1_rank2) *adc1_rank2 = sampling->adc1_j2_raw;
  if (adc2_rank1) *adc2_rank1 = sampling->adc2_j1_raw;
  if (adc2_rank2) *adc2_rank2 = sampling->adc2_j2_raw;
  if (adc2_rank3) *adc2_rank3 = sampling->adc2_j3_raw;
}
