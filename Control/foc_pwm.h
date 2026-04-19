#ifndef __FOC_PWM_H__
#define __FOC_PWM_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "foc_types.h"

/* PWM 模块接口。
 *
 * 负责管理三相占空比、采样窗口与 TIM1 触发点信息。
 */

typedef struct
{
  uint8_t svpwm_sector;
  float svpwm_da;
  float svpwm_db;
  float svpwm_dc;
  uint16_t ccr_u;
  uint16_t ccr_v;
  uint16_t ccr_w;
  uint16_t adc_win1_ccr;
  uint16_t adc_win2_ccr;
  uint16_t adc_sample_ccr;
  uint8_t adc_trigger_fallback;
  uint8_t midpoint_sampling;
} FOC_PwmData_t;
void FOC_PwmModule_Init(FOC_PwmData_t *pwm);
void FOC_PwmModule_SetTim1Mid(FOC_PwmData_t *pwm);
void FOC_PwmModule_ApplySvpwm(FOC_PwmData_t *pwm, float valpha, float vbeta, float vbus_v);
void FOC_PwmModule_GetSvpwmDebug(const FOC_PwmData_t *pwm, uint8_t *sector, float *duty_u, float *duty_v, float *duty_w);

void FOC_PwmModule_GetSamplingWindowDebug(const FOC_PwmData_t *pwm,
                                          uint8_t *sector,
                                          uint16_t *ccr_u,
                                          uint16_t *ccr_v,
                                          uint16_t *ccr_w,
                                          uint16_t *win1,
                                          uint16_t *win2,
                                          uint16_t *sample_ccr,
                                          uint8_t *used_fallback,
                                          uint8_t *trusted_pair);
#ifdef __cplusplus
}

#endif
#endif
