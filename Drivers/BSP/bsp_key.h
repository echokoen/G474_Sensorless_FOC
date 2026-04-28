/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_key.h
  * @brief   Simple key BSP for Start/Stop button.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __BSP_KEY_H__
#define __BSP_KEY_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include <stdint.h>

void BSP_KEY_Init(void);
void BSP_KEY_Task(void);
void BSP_KEY_SetSpeedTargetHz(float mech_hz);
uint8_t BSP_KEY_ConsumeStartStopRequest(void);
void BSP_KEY_EXTI_Callback(uint16_t GPIO_Pin);
#ifdef __cplusplus
}
#endif
#endif /* __BSP_KEY_H__ */
