/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_adc.h
  * @brief   ADC 底层驱动头文件
  *
  * @details
  * 当前工程的 ADC 分工如下：
  * 1. ADC1 仅负责规则组母线电压采样。
  *    - 通道：PA3 / ADC1_IN4
  *    - 采样对象：VBUS
  *    - 使用方式：连续规则转换，在中频任务中直接读取最近一次结果。
  *
  * 2. ADC2 仅负责注入组两相电流采样。
  *    - 通道：PC1 / ADC12_IN7 -> U 相电流
  *    - 通道：PC2 / ADC12_IN8 -> V 相电流
  *    - 触发源：TIM1_TRGO
  *    - 使用方式：每个 PWM 周期由定时器同步触发，ADC2 注入转换完成后进入高频控制。
  *
  * 这样分工的目的，是把“高频电流链路”和“中频母线电压链路”拆到两个 ADC 中，
  * 避免 ADC1 规则组和 ADC1 注入组共用时互相影响。
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ADC1: 规则组母线电压采样句柄 */
extern ADC_HandleTypeDef hadc1;

/* ADC2: 注入组两相电流采样句柄 */
extern ADC_HandleTypeDef hadc2;

/* ADC1 初始化：仅配置规则组 VBUS */
void MX_ADC1_Init(void);

/* ADC2 初始化：仅配置注入组 U/V 两相电流 */
void MX_ADC2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
