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

/* 故障原因位图。
 *
 * 可组合：
 * - 欠压 / 过压；
 * - 三相任一相过流。
 */
typedef enum
{
  FOC_FAULT_NONE         = 0u,
  FOC_FAULT_VBUS_UNDER   = (1u << 0),
  FOC_FAULT_VBUS_OVER    = (1u << 1),
  FOC_FAULT_OC_U         = (1u << 2),
  FOC_FAULT_OC_V         = (1u << 3),
  FOC_FAULT_OC_W         = (1u << 4)
} FOC_FaultFlags_t;

uint32_t FOC_ProtectionCheckFault(const FOC_SamplingData_t *sampling);

/* 执行统一保护停机：
 * - PWM 回中点；
 * - 关闭驱动使能；
 * - 状态切到 FAULT。
 */
void FOC_ProtectionApplyStop(FOC_StateTypeDef *state, FOC_PwmData_t *pwm);
#ifdef __cplusplus
}

#endif
#endif
