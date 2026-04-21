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

  if (sampling->vbus_v < FOC_VBUS_UNDERVOLTAGE_V)
  {
    fault_flags |= FOC_FAULT_VBUS_UNDER;
  }

  if (sampling->vbus_v > FOC_VBUS_OVERVOLTAGE_V)
  {
    fault_flags |= FOC_FAULT_VBUS_OVER;
  }

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

  return fault_flags;
}

void FOC_ProtectionApplyStop(FOC_StateTypeDef *state, FOC_PwmData_t *pwm)
{
  FOC_PwmModule_SetTim1Mid(pwm);
  HAL_GPIO_WritePin(M1_EN_DRIVER_GPIO_Port, M1_EN_DRIVER_Pin, GPIO_PIN_RESET);

  if (state != 0)
  {
    *state = FOC_STATE_FAULT;
  }
}
