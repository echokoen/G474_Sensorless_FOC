#ifndef __FOC_PWM_H__
#define __FOC_PWM_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "foc_types.h"

/* PWM / SVPWM 模块接口。
 *
 * 本模块是电压命令进入功率级之前的最后一层软件逻辑。
 * 上层给的是 alpha/beta 电压命令，本模块负责：
 * - 判断 SVPWM 扇区；
 * - 计算 T1/T2/T0；
 * - 生成三相 duty 和 TIM1 CCR；
 * - 维护 ADC 注入触发点 CCR4；
 * - 暴露采样窗口和 PWM 调试信息。
 *
 * 注意：
 * - 本模块不做电流采样；
 * - 本模块不做 observer；
 * - sample_sector 给采样模块使用，因此 sector 映射必须和 foc_sampling.c 保持一致。
 */

typedef struct
{
  /* 当前 SVPWM 扇区，范围 1~6。采样模块会按该扇区选择可信两相。 */
  uint8_t svpwm_sector;
  /* U/V/W 三相 duty，范围理论上为 0~1。 */
  float svpwm_da;
  float svpwm_db;
  float svpwm_dc;
  /* 扇区式 SVPWM 的两个有效矢量时间和零矢量时间，单位为 CCR tick。 */
  float svpwm_t1;
  float svpwm_t2;
  float svpwm_t0;
  /* 实际写入 TIM1 CH1/CH2/CH3 的比较值。 */
  uint16_t ccr_u;
  uint16_t ccr_v;
  uint16_t ccr_w;
  /* 当前可信两相相对 PWM 中点的窗口距离，用于判断中点采样是否安全。 */
  uint16_t adc_win1_ccr;
  uint16_t adc_win2_ccr;
  /* ADC 注入触发比较值，当前通常为 PWM 中点。 */
  uint16_t adc_sample_ccr;
  /* 采样触发是否 fallback：1=本拍可信两相低边公共导通窗口不足。 */
  uint8_t adc_trigger_fallback;
  /* 1=当前采样点为 PWM 中点；0=当前采样点已经按窗口动态偏移。 */
  uint8_t midpoint_sampling;
  /* 1=当前采样点落在可信两相低边公共导通安全窗口内。 */
  uint8_t adc_window_ok;
  /* 1=当前采样点由动态窗口算法偏移。 */
  uint8_t adc_dynamic_sampling;
} FOC_PwmData_t;

/* 初始化 PWM 数据结构，不直接启动硬件输出。 */
void FOC_PwmModule_Init(FOC_PwmData_t *pwm);

/* 强制三相回到中点占空比，常用于 IDLE/OFFSET/FAULT。 */
void FOC_PwmModule_SetTim1Mid(FOC_PwmData_t *pwm);

/* 根据 alpha/beta 电压命令计算并写入三相 PWM。 */
void FOC_PwmModule_ApplySvpwm(FOC_PwmData_t *pwm, float valpha, float vbeta, float vbus_v);

/* 读取 SVPWM duty 调试信息。 */
void FOC_PwmModule_GetSvpwmDebug(const FOC_PwmData_t *pwm, uint8_t *sector, float *duty_u, float *duty_v, float *duty_w);

/* 读取采样窗口调试信息，主要用于观察中点采样是否踩到窄窗口。 */
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
