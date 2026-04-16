#ifndef __FOC_BRIDGE_H__
#define __FOC_BRIDGE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bsp_adc.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"
#include "foc_config.h"

typedef enum
{
  FOC_STATE_IDLE = 0,
  FOC_STATE_OFFSET_CALIB,
  FOC_STATE_ALIGN,
  FOC_STATE_OPENLOOP,
  FOC_STATE_FAULT
} FOC_StateTypeDef;

void FOC_BridgeInit(void);
void FOC_Start(void);
void FOC_Stop(void);
void FOC_TaskHighFreq(void);
void FOC_TaskBackground(void);
FOC_StateTypeDef FOC_GetState(void);

/* 第一阶段核心步骤拆分。
 * 为什么这样拆：让每个阶段职责单一，便于先验证采样、对齐、开环和保护。 */
void FOC_CurrentOffsetCalib(void);
void FOC_AlignRun(void);
void FOC_OpenLoopRun(void);
void FOC_FaultStop(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_BRIDGE_H__ */
