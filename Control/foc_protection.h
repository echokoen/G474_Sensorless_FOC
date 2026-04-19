#ifndef __FOC_PROTECTION_H__
#define __FOC_PROTECTION_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "foc_types.h"
#include "foc_pwm.h"
#include "foc_sampling.h"

/* 保护模块接口。
 *
 * 提供故障检测与停机封装接口。
 */

uint8_t FOC_ProtectionCheckFault(const FOC_SamplingData_t *sampling);
void FOC_ProtectionApplyStop(FOC_StateTypeDef *state, FOC_PwmData_t *pwm);
#ifdef __cplusplus
}

#endif
#endif
