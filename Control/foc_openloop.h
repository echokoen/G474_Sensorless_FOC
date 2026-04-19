#ifndef __FOC_OPENLOOP_H__
#define __FOC_OPENLOOP_H__
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

/* 开环轨迹模块接口。
 *
 * 用于维护对齐阶段和开环阶段的角度、频率与推进节拍。
 */

typedef struct
{
  uint32_t align_ticks;
  float mech_freq_hz;
  float theta_e_rad;
} FOC_OpenLoopData_t;
void FOC_OpenLoopInit(FOC_OpenLoopData_t *openloop);
void FOC_OpenLoopResetAlign(FOC_OpenLoopData_t *openloop);
void FOC_OpenLoopResetRun(FOC_OpenLoopData_t *openloop);
void FOC_OpenLoopAdvance(FOC_OpenLoopData_t *openloop);
float FOC_OpenLoopGetElecSpeedRadPerSec(const FOC_OpenLoopData_t *openloop);
float FOC_OpenLoopGetFreqHz(const FOC_OpenLoopData_t *openloop);
float FOC_OpenLoopGetThetaErad(const FOC_OpenLoopData_t *openloop);
#ifdef __cplusplus
}

#endif
#endif
