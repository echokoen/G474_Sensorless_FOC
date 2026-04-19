/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_key.c
  * @brief   Start/Stop key processing.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_key.h"
#include "app_foc.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#define BSP_KEY_DEBOUNCE_MS    (300u)

static uint32_t g_key_last_tick = 0u;

void BSP_KEY_Init(void)
{
    g_key_last_tick = 0u;
}

void BSP_KEY_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now_tick;

    if (GPIO_Pin != Start_Stop_Pin)
    {
        return;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - g_key_last_tick) < BSP_KEY_DEBOUNCE_MS)
    {
        return;
    }

    if (HAL_GPIO_ReadPin(Start_Stop_GPIO_Port, Start_Stop_Pin) != GPIO_PIN_RESET)
    {
        return;
    }

    g_key_last_tick = now_tick;

    /* 直接依据当前状态决定启停，避免上电自动启动后状态不同步。 */
    if (AppFoc_GetState() == FOC_STATE_IDLE)
    {
        AppFoc_Start();
        printf("[KEY] AppFoc_Start()\r\n");
    }
    else
    {
        AppFoc_Stop();
        printf("[KEY] AppFoc_Stop()\r\n");
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BSP_KEY_EXTI_Callback(GPIO_Pin);
}
