/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   中断服务函数实现。
  *
  * @details
  * 本工程的高频 FOC 主链路由 ADC1_2_IRQn 驱动：
  * 1. ADC2 注入转换完成后进入 HAL 回调；
  * 2. 回调内部调用 AppFoc_HighFreqTask();
  * 3. 由高频任务完成采样更新、控制计算与 PWM 输出。
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
#include "app_foc.h"
/* USER CODE END EV */

/* Cortex-M4 Processor Interruption and Exception Handlers -------------------*/
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

/* STM32G4xx Peripheral Interrupt Handlers -----------------------------------*/

/**
  * @brief DMA1 Channel1 中断。
  * @note  当前用于 USART1 RX DMA。
  */
void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/**
  * @brief ADC1_2 共享中断。
  *
  * @details
  * 本工程只使用 hadc2 的注入转换完成回调作为高频 FOC 调度入口。
  */
void ADC1_2_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc2);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc2)
  {
    AppFoc_HighFreqTask();
  }
}

/**
  * @brief TIM1 Break / TIM15 中断。
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief TIM1 Update / TIM16 中断。
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief EXTI line[15:10] 中断。
  * @note  Start/Stop 按键连接在 PE14，并映射到 EXTI15_10_IRQn。
  */
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(Start_Stop_Pin);
}
