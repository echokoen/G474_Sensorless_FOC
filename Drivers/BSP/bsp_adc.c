/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @brief   ADC �ײ�����ʵ��
  *
  * @details
  * ��ǰ�����е� ADC ��Դ�����ֿܷ�ʹ�ã�
  *
  * 1. ADC1 -> ������ -> ĸ�ߵ�ѹ VBUS
  *    - ʹ�� PA3 / ADC1_IN4
  *    - ��������ת��
  *    - ��Ƶ����ͨ�� HAL_ADC_GetValue(&hadc1) ��ȡ���һ�ι���ת�����
  *
  * 2. ADC2 -> ע���� -> U/V �������
  *    - ʹ�� PC1 / ADC12_IN7 �� U �����
  *    - ʹ�� PC2 / ADC12_IN8 �� V �����
  *    - �� TIM1_TRGO ͬ������ע��ת��
  *    - ADC2 ע����ת����ɺ󴥷��жϣ��ڻص��н����Ƶ������·
  *
  * ������Ƶ�Ŀ�ģ�
  * - ������������ FOC ��Ƶ�ؼ���·��Ҫ��ʵʱ���ȶ������ܱ��������ϡ�
  * - ĸ�ߵ�ѹֻ����Ƶ��ȡ������Ҫռ��ע���飬Ҳ����Ҫ�͵�����������ͬһ�� ADC��
  ******************************************************************************
  */
/* USER CODE END Header */
#include "bsp_adc.h"

/* ADC1 �����飺ĸ�ߵ�ѹ */
ADC_HandleTypeDef hadc1;

/* ADC2 ע���飺U/V ������� */
ADC_HandleTypeDef hadc2;


/**
  * @brief  ��ʼ�� ADC1
  * @note   ADC1 �ڵ�ǰ������ֻ����ĸ�ߵ�ѹ�����������
  *         ���ٳе�����ע�������񣬱���͸�Ƶ����������·������š�
  */
void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /*
   * ADC1 ����������˵����
   * 1. ��ͨ��������
   * 2. ��������
   * 3. ����ת��ʹ��
   * 4. ��Ƶ������ֱ�Ӷ�ȡ���һ��ת���������
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * ������ͨ�����ã�
   * PA3 -> ADC1_IN4 -> ĸ�ߵ�ѹ VBUS
   */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  ��ʼ�� ADC2
  * @note   ADC2 �ڵ�ǰ������ֻ����ע�����������������
  *         ע������ TIM1_TRGO �������� PWM ����ͬ����
  */
void MX_ADC2_Init(void)
{
    ADC_InjectionConfTypeDef sConfigInjected = {0};


    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.GainCompensation = 0;
    hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc2.Init.LowPowerAutoWait = DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.NbrOfConversion = 1;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc2.Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(&hadc2) != HAL_OK)
    {

        Error_Handler();
    }


    sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    sConfigInjected.InjectedOffset = 0;
    sConfigInjected.InjectedNbrOfConversion = 2;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.QueueInjectedContext = DISABLE;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    sConfigInjected.InjecOversamplingMode = DISABLE;

    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
    {
        Error_Handler();
    }


    sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
    {
        Error_Handler();
    }

}

/* ADC1 �� ADC2 ���� ADC12 ʱ�ӣ���������ü�����ʽ����ʱ�ӿ��� */
static uint32_t HAL_RCC_ADC12_CLK_ENABLED = 0;

/**
  * @brief  ADC MSP ��ʼ��
  * @param  adcHandle: ָ�� ADC ���
  * @note   ������Ҫ��ɣ�
  *         1. ADC12 ����ʱ������
  *         2. GPIO ģ����������
  *         3. ADC1_2 �����ж�����
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    if ((adcHandle->Instance == ADC1) || (adcHandle->Instance == ADC2))
    {
        HAL_RCC_ADC12_CLK_ENABLED++;
        if (HAL_RCC_ADC12_CLK_ENABLED == 1)
        {
            PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
            PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_PLL;
            if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
            {
                Error_Handler();
            }

            __HAL_RCC_ADC12_CLK_ENABLE();
        }

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        if (adcHandle->Instance == ADC1)
        {
            GPIO_InitStruct.Pin = M1_BUS_VOLTAGE_Pin;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(M1_BUS_VOLTAGE_GPIO_Port, &GPIO_InitStruct);
        }
        else
        {
            GPIO_InitStruct.Pin = M1_CURR_AMPL_U_Pin | M1_CURR_AMPL_V_Pin;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        }

        HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    }
}

/**
  * @brief  ADC MSP ����ʼ��
  * @param  adcHandle: ָ�� ADC ���
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{
  if ((adcHandle->Instance == ADC1) || (adcHandle->Instance == ADC2))
  {
    HAL_RCC_ADC12_CLK_ENABLED--;
    if (HAL_RCC_ADC12_CLK_ENABLED == 0)
    {
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    if (adcHandle->Instance == ADC1)
    {
      HAL_GPIO_DeInit(M1_BUS_VOLTAGE_GPIO_Port, M1_BUS_VOLTAGE_Pin);
    }
    else
    {
      HAL_GPIO_DeInit(GPIOC, M1_CURR_AMPL_U_Pin | M1_CURR_AMPL_V_Pin);
    }

    /*
     * ADC1_2 �ǹ����жϡ�
     * һ�㲻�����ڵ��� ADC ����ʼ��ʱóȻ�ر� NVIC��
     * �������Ӱ����һ�� ADC �Ĺ�����
     */
  }
}
