#ifndef __FOC_SAMPLING_H__
#define __FOC_SAMPLING_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "foc_types.h"

/* 采样聚合模块接口。
 *
 * 用于维护 ADC 原始值、母线电压、相电流以及偏置校准状态。
 * 当前主线固定采用扇区重构采样，不再保留 direct 三相采样开关。
 */

typedef struct
{
  float iu_offset_v;
  float iv_offset_v;
  float iw_offset_v;
  float vbus_v;
  float iu_a;
  float iv_a;
  float iw_a;
  uint16_t raw_u;
  uint16_t raw_v;
  uint16_t raw_w;
  uint16_t adc1_j1_raw;
  uint16_t adc1_j2_raw;
  uint16_t adc2_j1_raw;
  uint16_t adc2_j2_raw;
  uint16_t adc2_j3_raw;
  uint8_t sample_sector;
  uint8_t trusted_current_pair; /* 当前扇区下用于重构的可信两相编号。 */
  uint32_t offset_samples;
  float offset_sum_u;
  float offset_sum_v;
  float offset_sum_w;
  uint8_t adc1_ready;
} FOC_SamplingData_t;
void FOC_SamplingModule_Init(FOC_SamplingData_t *sampling);
void FOC_SamplingModule_SetAdc1Ready(FOC_SamplingData_t *sampling, uint8_t ready);
void FOC_SamplingModule_SetSampleSector(FOC_SamplingData_t *sampling, uint8_t sector);
void FOC_SamplingModule_Update(FOC_SamplingData_t *sampling);
void FOC_SamplingModule_RunOffsetCalib(FOC_SamplingData_t *sampling, FOC_StateTypeDef *state);
float FOC_SamplingModule_GetBusVoltageV(const FOC_SamplingData_t *sampling);
float FOC_SamplingModule_GetPhaseCurrentUa(const FOC_SamplingData_t *sampling);
float FOC_SamplingModule_GetPhaseCurrentVa(const FOC_SamplingData_t *sampling);
float FOC_SamplingModule_GetPhaseCurrentWa(const FOC_SamplingData_t *sampling);
void FOC_SamplingModule_GetCurrentOffsets(const FOC_SamplingData_t *sampling, float *iu_offset_v, float *iv_offset_v, float *iw_offset_v);
void FOC_SamplingModule_GetCurrentRawAdc(const FOC_SamplingData_t *sampling, uint16_t *raw_u, uint16_t *raw_v, uint16_t *raw_w);

void FOC_SamplingModule_GetInjectedRawAdcDebug(const FOC_SamplingData_t *sampling,
                                               uint16_t *adc1_rank1,
                                               uint16_t *adc1_rank2,
                                               uint16_t *adc2_rank1,
                                               uint16_t *adc2_rank2,
                                               uint16_t *adc2_rank3);
#ifdef __cplusplus
}

#endif
#endif
