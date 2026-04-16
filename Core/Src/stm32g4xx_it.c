/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   ?§Ø?????????
  *
  * @details
  * ??????????? FOC ????????§Ø?????? ADC1_2_IRQn??
  *
  * 1. ADC1 ?? ADC2 ???? ADC1_2_IRQn ?§Ø???????
  * 2. ????????????¡¤?? ADC2 ?????????§Ø???????
  * 3. ADC1 ??????????????????????????????????????§Ø????¨¢?
  *
  * ?????? ADC1_2_IRQHandler() ?§µ?????·Ú???????? hadc2??
  * ??????¨¢?ADC2 ????? -> ???????? -> ??????????????¡¤???????
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
#include "foc_bridge.h"
/* USER CODE END EV */

/******************************************************************************
//           Cortex-M4 Processor Interruption and Exception Handlers          
/******************************************************************************/
void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************
// STM32G4xx Peripheral Interrupt Handlers                                    
/******************************************************************************/

/**
  * @brief DMA1 ??? 1 ?§Ø?
  * @note  ??????? USART1 DMA ?????
  */
void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/**
  * @brief ADC1_2 ?????§Ø?
  *
  * @details
  * ??? ADC1 ?? ADC2 ?????????§Ø?????????????????§µ?
  * - ADC1 ????????????????????????????????????
  * - ADC2 ??????? U/V ?????????????????????????????????
  *
  * ???????????? hadc2??????????¡¤??????
  * ADC2 injected complete -> HAL ??? -> ???????? -> ???????
  */
void ADC1_2_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc2);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc2)
  {
    FOC_TaskHighFreq();
  }
}

/**
  * @brief TIM1 ????§Ø?? TIM15 ????§Ø?
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief TIM1 ?????§Ø?? TIM16 ????§Ø?
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}
/**
  * @brief EXTI line[15:10] interrupt handler
  * @note  Start/Stop key is on PE14, routed to EXTI15_10_IRQn.
  */
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(Start_Stop_Pin);
}