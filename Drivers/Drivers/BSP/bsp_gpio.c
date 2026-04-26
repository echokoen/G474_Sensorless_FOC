/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "bsp_gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*
     * Ĭ��״̬��
     * 1. PC13(M1_EN_DRIVER) ���ͣ��ر�����
     * 2. PB12(M1_BRAKE)     ���ߣ��̶��ͷ�ɲ��
     *
     * ��ǰ���̲����� TIM1 BKIN Ӳ��ɲ�����ܣ�
     * PB12 ����Ϊ��ͨ GPIO ��������ָߵ�ƽ��
     */
    HAL_GPIO_WritePin(M1_EN_DRIVER_GPIO_Port, M1_EN_DRIVER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_BRAKE_GPIO_Port, M1_BRAKE_Pin, GPIO_PIN_SET);

    /* PC13 : ����ʹ�ܣ����߿������͹� */
    GPIO_InitStruct.Pin = M1_EN_DRIVER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(M1_EN_DRIVER_GPIO_Port, &GPIO_InitStruct);

    /* PB12 : ��ͨ GPIO ������̶����ָߵ�ƽ�ͷ� */
    GPIO_InitStruct.Pin = M1_BRAKE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(M1_BRAKE_GPIO_Port, &GPIO_InitStruct);

    /* Start/Stop ���� */
    GPIO_InitStruct.Pin = Start_Stop_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(Start_Stop_GPIO_Port, &GPIO_InitStruct);

    /* KEY0 / KEY1 : 轮询输入，用于调速。 */
    GPIO_InitStruct.Pin = KEY0_Pin | KEY1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* EXTI interrupt init */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
