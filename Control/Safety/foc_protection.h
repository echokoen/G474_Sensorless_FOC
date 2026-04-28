#ifndef __FOC_PROTECTION_H__
#define __FOC_PROTECTION_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "foc_types.h"
#include "foc_current_sense.h"

/* 保护模块接口。
 *
 * 只提供故障检测接口，停机动作由 App 层统一执行。
 */

/* 故障原因位图。
 *
 * 可组合：
 * - 欠压 / 过压；
 * - 三相任一相过流；
 * - 采样 hold-last 超限。
 */
typedef enum
{
  FOC_FAULT_NONE         = 0u,
  FOC_FAULT_VBUS_UNDER   = (1u << 0),
  FOC_FAULT_VBUS_OVER    = (1u << 1),
  FOC_FAULT_OC_U         = (1u << 2),
  FOC_FAULT_OC_V         = (1u << 3),
  FOC_FAULT_OC_W         = (1u << 4),
  FOC_FAULT_SAMPLE_HOLD  = (1u << 5)
} FOC_FaultFlags_t;

uint32_t FOC_ProtectionCheckFault(const FOC_CurrentSense_t *current_sense);
#ifdef __cplusplus
}

#endif
#endif
