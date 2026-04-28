#ifndef FOC_CURRENT_SENSE_H
#define FOC_CURRENT_SENSE_H
#ifdef __cplusplus
extern "C" {
#endif
#include "foc_types.h"

/* 采样聚合模块接口。
 *
 * 用于维护 ADC 原始值、母线电压、相电流以及偏置校准状态。
 * 当前主线固定采用扇区重构采样，不再保留 direct 三相采样开关。
 *
 * 当前电流采样策略：
 * - 三下桥电阻采样；
 * - ADC 一次触发同时采三相原始值；
 * - 根据 SVPWM 扇区选择可信两相；
 * - 第三相使用 iu + iv + iw = 0 重构；
 * - offset 校准阶段只累计零电流偏置，不进入闭环控制。
 */

typedef struct
{
  /* 三相电流采样零点，单位 V。offset 校准完成后写入。 */
  float iu_offset_v;
  float iv_offset_v;
  float iw_offset_v;
  /* 当前母线电压，单位 V。 */
  float vbus_v;
  /* 重构后的三相电流，单位 A。 */
  float iu_a;
  float iv_a;
  float iw_a;
  /* 最近一次有效电流。采样窗口不足时用于 hold-last，避免坏样本进入电流环。 */
  float last_valid_iu_a;
  float last_valid_iv_a;
  float last_valid_iw_a;
  uint8_t last_valid_current_ready;
  uint8_t fallback_request;
  uint8_t used_hold_last_current;
  uint16_t fallback_hold_count;
  /* 当前拍三相原始 ADC 值，用于串口诊断。 */
  uint16_t raw_u;
  uint16_t raw_v;
  uint16_t raw_w;
  /* 注入通道调试缓存，保留 ADC1/ADC2 各 rank 原始值。 */
  uint16_t adc1_j1_raw;
  uint16_t adc1_j2_raw;
  uint16_t adc2_j1_raw;
  uint16_t adc2_j2_raw;
  uint16_t adc2_j3_raw;
  /* 当前采样数据按哪个 SVPWM 扇区解释。 */
  uint8_t sample_sector;
  uint8_t trusted_current_pair; /* 当前扇区下用于重构的可信两相编号。 */
  /* offset 校准累计状态。 */
  uint32_t offset_samples;
  float offset_sum_u;
  float offset_sum_v;
  float offset_sum_w;
  /* ADC1 常规采样是否可用；不可用时母线电压给默认值。 */
  uint8_t adc1_ready;
} FOC_CurrentSense_t;
void FOC_CurrentSense_Init(FOC_CurrentSense_t *current_sense);
void FOC_CurrentSense_SetAdc1Ready(FOC_CurrentSense_t *current_sense, uint8_t ready);
void FOC_CurrentSense_SetSampleSector(FOC_CurrentSense_t *current_sense, uint8_t sector);
void FOC_CurrentSense_SetFallbackRequest(FOC_CurrentSense_t *current_sense, uint8_t fallback);
void FOC_CurrentSense_ResetRuntime(FOC_CurrentSense_t *current_sense);
void FOC_CurrentSense_Update(FOC_CurrentSense_t *current_sense);
void FOC_CurrentSense_RunOffsetCalib(FOC_CurrentSense_t *current_sense, FOC_StateTypeDef *state);
float FOC_CurrentSense_GetBusVoltageV(const FOC_CurrentSense_t *current_sense);
float FOC_CurrentSense_GetPhaseCurrentUa(const FOC_CurrentSense_t *current_sense);
float FOC_CurrentSense_GetPhaseCurrentVa(const FOC_CurrentSense_t *current_sense);
float FOC_CurrentSense_GetPhaseCurrentWa(const FOC_CurrentSense_t *current_sense);
void FOC_CurrentSense_GetCurrentOffsets(const FOC_CurrentSense_t *current_sense, float *iu_offset_v, float *iv_offset_v, float *iw_offset_v);
void FOC_CurrentSense_GetCurrentRawAdc(const FOC_CurrentSense_t *current_sense, uint16_t *raw_u, uint16_t *raw_v, uint16_t *raw_w);

void FOC_CurrentSense_GetInjectedRawAdcDebug(const FOC_CurrentSense_t *current_sense,
                                               uint16_t *adc1_rank1,
                                               uint16_t *adc1_rank2,
                                               uint16_t *adc2_rank1,
                                               uint16_t *adc2_rank2,
                                               uint16_t *adc2_rank3);
#ifdef __cplusplus
}

#endif
#endif /* FOC_CURRENT_SENSE_H */
