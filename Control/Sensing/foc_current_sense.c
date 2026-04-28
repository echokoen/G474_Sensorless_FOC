#include "foc_current_sense.h"
#include "bsp_adc.h"
#include "foc_config.h"
#include "foc_math.h"

/* 采样聚合实现。
 *
 * 负责读取 ADC 原始值、换算相电流和母线电压，并维护零点校准。
 */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

static float foc_current_sense_adc_to_voltage(uint16_t raw)
{
  /* ADC 原始计数 -> 引脚电压。 */
  const float adc_lsb_v = FOC_ADC_VREF / FOC_ADC_FULL_SCALE;
  return (float)raw * adc_lsb_v;
}

static float foc_current_sense_current_from_adc(float vadc, float offset_v, float sign)
{
  /* 采样电压 -> 相电流：
   * I = (Vadc - Voffset) / (放大倍数 * 采样电阻)
   * sign 用于适配硬件放大器方向或相线定义。
   */
  const float inv_gain_shunt = 1.0f / (FOC_GAIN * FOC_SHUNT_OHM);
  return sign * (vadc - offset_v) * inv_gain_shunt;
}

static float foc_current_sense_adc_to_bus_voltage(uint16_t raw)
{
  return foc_current_sense_adc_to_voltage(raw) * FOC_VBUS_RATIO;
}

static void foc_current_sense_cache_injected_raw_debug(FOC_CurrentSense_t *current_sense,
                                                  uint16_t adc2_j1,
                                                  uint16_t adc2_j2,
                                                  uint16_t adc2_j3)
{
  current_sense->adc1_j1_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  current_sense->adc1_j2_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
  current_sense->adc2_j1_raw = adc2_j1;
  current_sense->adc2_j2_raw = adc2_j2;
  current_sense->adc2_j3_raw = adc2_j3;
}

static void foc_current_sense_update_vbus(FOC_CurrentSense_t *current_sense)
{
  if (current_sense->adc1_ready == 0u)
  {
    current_sense->vbus_v = 24.0f;
    return;
  }

  current_sense->vbus_v = foc_current_sense_adc_to_bus_voltage((uint16_t)HAL_ADC_GetValue(&hadc1));
}

static void foc_current_sense_update_currents(FOC_CurrentSense_t *current_sense)
{
  /* 读取 ADC2 注入 rank。
   * 当前约定：
   * rank1 -> U，rank2 -> V，rank3 -> W。
   * 如果 CubeMX 通道顺序变化，这里必须同步更新。
   */
  const uint16_t raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
  const uint16_t raw_v = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
  const uint16_t raw_w = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
  const float vu = foc_current_sense_adc_to_voltage(raw_u);
  const float vv = foc_current_sense_adc_to_voltage(raw_v);
  const float vw = foc_current_sense_adc_to_voltage(raw_w);
  const float iu_adc = foc_current_sense_current_from_adc(vu, current_sense->iu_offset_v, FOC_CURR_SIGN_U);
  const float iv_adc = foc_current_sense_current_from_adc(vv, current_sense->iv_offset_v, FOC_CURR_SIGN_V);
  const float iw_adc = foc_current_sense_current_from_adc(vw, current_sense->iw_offset_v, FOC_CURR_SIGN_W);
  float iu_new = 0.0f;
  float iv_new = 0.0f;
  float iw_new = 0.0f;

  foc_current_sense_cache_injected_raw_debug(current_sense, raw_u, raw_v, raw_w);
  current_sense->raw_u = raw_u;
  current_sense->raw_v = raw_v;
  current_sense->raw_w = raw_w;

  switch (current_sense->sample_sector)
  {
    case 1u:
    case 6u:
      /* 扇区 1/6：V/W 两相更可信，U 相用三相和为 0 重构。 */
      current_sense->trusted_current_pair = 1u;
      iv_new = iv_adc;
      iw_new = iw_adc;
      iu_new = -(iv_new + iw_new);
      break;

    case 2u:
    case 3u:
      /* 扇区 2/3：U/W 两相更可信，V 相重构。 */
      current_sense->trusted_current_pair = 2u;
      iu_new = iu_adc;
      iw_new = iw_adc;
      iv_new = -(iu_new + iw_new);
      break;

    case 4u:
    case 5u:
    default:
      /* 扇区 4/5：U/V 两相更可信，W 相重构。 */
      current_sense->trusted_current_pair = 3u;
      iu_new = iu_adc;
      iv_new = iv_adc;
      iw_new = -(iu_new + iv_new);
      break;
  }

#if (FOC_ADC_HOLD_LAST_ON_FALLBACK != 0u)
  if ((current_sense->fallback_request != 0u) && (current_sense->last_valid_current_ready != 0u))
  {
    current_sense->iu_a = current_sense->last_valid_iu_a;
    current_sense->iv_a = current_sense->last_valid_iv_a;
    current_sense->iw_a = current_sense->last_valid_iw_a;
    current_sense->used_hold_last_current = 1u;
    if (current_sense->fallback_hold_count < 0xFFFFu)
    {
      current_sense->fallback_hold_count++;
    }
    return;
  }
#endif

  current_sense->fallback_hold_count = 0u;
  current_sense->iu_a = iu_new;
  current_sense->iv_a = iv_new;
  current_sense->iw_a = iw_new;
  current_sense->last_valid_iu_a = iu_new;
  current_sense->last_valid_iv_a = iv_new;
  current_sense->last_valid_iw_a = iw_new;
  current_sense->last_valid_current_ready = 1u;
  current_sense->used_hold_last_current = 0u;
}

void FOC_CurrentSense_Init(FOC_CurrentSense_t *current_sense)
{
  if (current_sense == 0)
  {
    return;
  }

  current_sense->iu_offset_v = 0.0f;
  current_sense->iv_offset_v = 0.0f;
  current_sense->iw_offset_v = 0.0f;
  current_sense->vbus_v = 24.0f;
  current_sense->iu_a = 0.0f;
  current_sense->iv_a = 0.0f;
  current_sense->iw_a = 0.0f;
  current_sense->last_valid_iu_a = 0.0f;
  current_sense->last_valid_iv_a = 0.0f;
  current_sense->last_valid_iw_a = 0.0f;
  current_sense->last_valid_current_ready = 0u;
  current_sense->fallback_request = 0u;
  current_sense->used_hold_last_current = 0u;
  current_sense->fallback_hold_count = 0u;
  current_sense->raw_u = 0u;
  current_sense->raw_v = 0u;
  current_sense->raw_w = 0u;
  current_sense->adc1_j1_raw = 0u;
  current_sense->adc1_j2_raw = 0u;
  current_sense->adc2_j1_raw = 0u;
  current_sense->adc2_j2_raw = 0u;
  current_sense->adc2_j3_raw = 0u;
  current_sense->sample_sector = 1u;
  current_sense->trusted_current_pair = 0u;
  current_sense->offset_samples = 0u;
  current_sense->offset_sum_u = 0.0f;
  current_sense->offset_sum_v = 0.0f;
  current_sense->offset_sum_w = 0.0f;
  current_sense->adc1_ready = 0u;
}

void FOC_CurrentSense_SetAdc1Ready(FOC_CurrentSense_t *current_sense, uint8_t ready)
{
  if (current_sense != 0)
  {
    current_sense->adc1_ready = ready;
  }
}

void FOC_CurrentSense_SetSampleSector(FOC_CurrentSense_t *current_sense, uint8_t sector)
{
  if (current_sense != 0)
  {
    current_sense->sample_sector = sector;
  }
}

void FOC_CurrentSense_SetFallbackRequest(FOC_CurrentSense_t *current_sense, uint8_t fallback)
{
  if (current_sense != 0)
  {
    current_sense->fallback_request = fallback;
  }
}

void FOC_CurrentSense_ResetRuntime(FOC_CurrentSense_t *current_sense)
{
  if (current_sense == 0)
  {
    return;
  }

  current_sense->iu_a = 0.0f;
  current_sense->iv_a = 0.0f;
  current_sense->iw_a = 0.0f;
  current_sense->last_valid_iu_a = 0.0f;
  current_sense->last_valid_iv_a = 0.0f;
  current_sense->last_valid_iw_a = 0.0f;
  current_sense->last_valid_current_ready = 0u;
  current_sense->fallback_request = 0u;
  current_sense->used_hold_last_current = 0u;
  current_sense->fallback_hold_count = 0u;
}

void FOC_CurrentSense_Update(FOC_CurrentSense_t *current_sense)
{
  if (current_sense == 0)
  {
    return;
  }

  foc_current_sense_update_vbus(current_sense);
  foc_current_sense_update_currents(current_sense);
}

void FOC_CurrentSense_RunOffsetCalib(FOC_CurrentSense_t *current_sense, FOC_StateTypeDef *state)
{
  const uint16_t raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
  const uint16_t raw_v = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
  const uint16_t raw_w = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);

  if (current_sense == 0)
  {
    return;
  }

  /* offset 校准要求电机无有效电流输出。
   * 这里只累计 ADC 电压平均值，作为后续电流换算零点。
   */
  foc_current_sense_cache_injected_raw_debug(current_sense, raw_u, raw_v, raw_w);
  current_sense->offset_sum_u += foc_current_sense_adc_to_voltage(raw_u);
  current_sense->offset_sum_v += foc_current_sense_adc_to_voltage(raw_v);
  current_sense->offset_sum_w += foc_current_sense_adc_to_voltage(raw_w);
  current_sense->offset_samples++;

  if (current_sense->offset_samples >= FOC_CURRENT_OFFSET_CALIB_SAMPLES)
  {
    const float inv_offset_samples = 1.0f / (float)current_sense->offset_samples;
    current_sense->iu_offset_v = current_sense->offset_sum_u * inv_offset_samples;
    current_sense->iv_offset_v = current_sense->offset_sum_v * inv_offset_samples;
    current_sense->iw_offset_v = current_sense->offset_sum_w * inv_offset_samples;

    if (state != 0)
    {
      *state = FOC_STATE_ALIGN;
    }
  }
}

float FOC_CurrentSense_GetBusVoltageV(const FOC_CurrentSense_t *current_sense)
{
  return (current_sense != 0) ? current_sense->vbus_v : 0.0f;
}

float FOC_CurrentSense_GetPhaseCurrentUa(const FOC_CurrentSense_t *current_sense)
{
  return (current_sense != 0) ? current_sense->iu_a : 0.0f;
}

float FOC_CurrentSense_GetPhaseCurrentVa(const FOC_CurrentSense_t *current_sense)
{
  return (current_sense != 0) ? current_sense->iv_a : 0.0f;
}

float FOC_CurrentSense_GetPhaseCurrentWa(const FOC_CurrentSense_t *current_sense)
{
  return (current_sense != 0) ? current_sense->iw_a : 0.0f;
}

void FOC_CurrentSense_GetCurrentOffsets(const FOC_CurrentSense_t *current_sense, float *iu_offset_v, float *iv_offset_v, float *iw_offset_v)
{
  if (current_sense == 0)
  {
    return;
  }

  if (iu_offset_v) *iu_offset_v = current_sense->iu_offset_v;
  if (iv_offset_v) *iv_offset_v = current_sense->iv_offset_v;
  if (iw_offset_v) *iw_offset_v = current_sense->iw_offset_v;
}

void FOC_CurrentSense_GetCurrentRawAdc(const FOC_CurrentSense_t *current_sense, uint16_t *raw_u, uint16_t *raw_v, uint16_t *raw_w)
{
  if (current_sense == 0)
  {
    return;
  }

  if (raw_u) *raw_u = current_sense->raw_u;
  if (raw_v) *raw_v = current_sense->raw_v;
  if (raw_w) *raw_w = current_sense->raw_w;
}

void FOC_CurrentSense_GetInjectedRawAdcDebug(const FOC_CurrentSense_t *current_sense,
                                                uint16_t *adc1_rank1,
                                                uint16_t *adc1_rank2,
                                                uint16_t *adc2_rank1,
                                                uint16_t *adc2_rank2,
                                                uint16_t *adc2_rank3)
{
  if (current_sense == 0)
  {
    return;
  }

  if (adc1_rank1) *adc1_rank1 = current_sense->adc1_j1_raw;
  if (adc1_rank2) *adc1_rank2 = current_sense->adc1_j2_raw;
  if (adc2_rank1) *adc2_rank1 = current_sense->adc2_j1_raw;
  if (adc2_rank2) *adc2_rank2 = current_sense->adc2_j2_raw;
  if (adc2_rank3) *adc2_rank3 = current_sense->adc2_j3_raw;
}
