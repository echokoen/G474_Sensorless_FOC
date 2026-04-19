#include "foc_protection.h"
#include "bsp_gpio.h"
#include "foc_config.h"
#include <math.h>

/* 保护逻辑实现。
 *
 * 负责基础阈值检测，并在故障发生时执行统一停机动作。
 */
uint8_t FOC_ProtectionCheckFault(const FOC_SamplingData_t *sampling)
{
  if (sampling == 0)
  {
    return 0u;
  }

  if ((sampling->vbus_v < FOC_VBUS_UNDERVOLTAGE_V) || (sampling->vbus_v > FOC_VBUS_OVERVOLTAGE_V))
  {
    return 1u;
  }

  if ((fabsf(sampling->iu_a) > FOC_CURRENT_LIMIT_A) ||

      (fabsf(sampling->iv_a) > FOC_CURRENT_LIMIT_A) ||

      (fabsf(sampling->iw_a) > FOC_CURRENT_LIMIT_A))
  {
    return 1u;
  }

  return 0u;
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
