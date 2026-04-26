#include "foc_protection.h"
#include "bsp_gpio.h"
#include "foc_config.h"
#include <math.h>

/* 保护逻辑实现。
 *
 * 负责基础阈值检测，并在故障发生时执行统一停机动作。
 */
uint32_t FOC_ProtectionCheckFault(const FOC_SamplingData_t *sampling)
{
  uint32_t fault_flags = FOC_FAULT_NONE;

  if (sampling == 0)
  {
    return FOC_FAULT_NONE;
  }

  /* 母线电压保护。
   * 欠压时 PWM 电压利用率和观测器输入都会失真；
   * 过压时先保护功率级。
   */
  if (sampling->vbus_v < FOC_VBUS_UNDERVOLTAGE_V)
  {
    fault_flags |= FOC_FAULT_VBUS_UNDER;
  }

  if (sampling->vbus_v > FOC_VBUS_OVERVOLTAGE_V)
  {
    fault_flags |= FOC_FAULT_VBUS_OVER;
  }

  /* 三相独立过流保护。
   * 这里用重构后的相电流做软件保护，硬件比较器/驱动保护应作为更底层兜底。
   */
  if (fabsf(sampling->iu_a) > FOC_CURRENT_LIMIT_A)
  {
    fault_flags |= FOC_FAULT_OC_U;
  }

  if (fabsf(sampling->iv_a) > FOC_CURRENT_LIMIT_A)
  {
    fault_flags |= FOC_FAULT_OC_V;
  }

  if (fabsf(sampling->iw_a) > FOC_CURRENT_LIMIT_A)
  {
    fault_flags |= FOC_FAULT_OC_W;
  }

#if (FOC_ADC_FALLBACK_HOLD_FAULT_ENABLE != 0u)
  if (sampling->fallback_hold_count > FOC_ADC_FALLBACK_HOLD_MAX_TICKS)
  {
    fault_flags |= FOC_FAULT_SAMPLE_HOLD;
  }
#endif

  return fault_flags;
}

void FOC_ProtectionApplyStop(FOC_StateTypeDef *state, FOC_PwmData_t *pwm)
{
  /* 故障停机顺序：
   * 1. 先把 PWM 比较值拉回安全中点；
   * 2. 再关闭驱动使能；
   * 3. 最后锁定状态机到 FAULT。
   */
  FOC_PwmModule_SetTim1Mid(pwm);
  HAL_GPIO_WritePin(M1_EN_DRIVER_GPIO_Port, M1_EN_DRIVER_Pin, GPIO_PIN_RESET);

  if (state != 0)
  {
    *state = FOC_STATE_FAULT;
  }
}
