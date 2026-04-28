#include "foc_protection.h"
#include "foc_config.h"
#include <math.h>

/* 保护逻辑实现。
 *
 * 只负责基础阈值检测，停机动作由 App 层统一执行。
 */
uint32_t FOC_ProtectionCheckFault(const FOC_CurrentSense_t *current_sense)
{
  uint32_t fault_flags = FOC_FAULT_NONE;

  if (current_sense == 0)
  {
    return FOC_FAULT_NONE;
  }

  /* 母线电压保护。
   * 欠压时 PWM 电压利用率和观测器输入都会失真；
   * 过压时先保护功率级。
   */
  if (current_sense->vbus_v < FOC_VBUS_UNDERVOLTAGE_V)
  {
    fault_flags |= FOC_FAULT_VBUS_UNDER;
  }

  if (current_sense->vbus_v > FOC_VBUS_OVERVOLTAGE_V)
  {
    fault_flags |= FOC_FAULT_VBUS_OVER;
  }

  /* 三相独立过流保护。
   * 这里用重构后的相电流做软件保护，硬件比较器/驱动保护应作为更底层兜底。
   */
  if (fabsf(current_sense->iu_a) > FOC_CURRENT_LIMIT_A)
  {
    fault_flags |= FOC_FAULT_OC_U;
  }

  if (fabsf(current_sense->iv_a) > FOC_CURRENT_LIMIT_A)
  {
    fault_flags |= FOC_FAULT_OC_V;
  }

  if (fabsf(current_sense->iw_a) > FOC_CURRENT_LIMIT_A)
  {
    fault_flags |= FOC_FAULT_OC_W;
  }

#if (FOC_ADC_FALLBACK_HOLD_FAULT_ENABLE != 0u)
  if (current_sense->fallback_hold_count > FOC_ADC_FALLBACK_HOLD_MAX_TICKS)
  {
    fault_flags |= FOC_FAULT_SAMPLE_HOLD;
  }
#endif

  return fault_flags;
}
