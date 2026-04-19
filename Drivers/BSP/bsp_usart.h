/* USER CODE BEGIN Header */
/**
  ******************************************************************************
 * @file    bsp_usart.h
  * @brief   This file contains all the function prototypes for
 *          the bsp_usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_USART_H__
#define __BSP_USART_H__
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef BSP_USART1_Write(const uint8_t *data, uint16_t len);
HAL_StatusTypeDef BSP_USART1_Print(const char *str);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /* __BSP_USART_H__ */
