#ifndef __FOC_OPENLOOP_H__
#define __FOC_OPENLOOP_H__
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

/* 开环轨迹模块接口。
 *
 * 用于维护对齐阶段和开环阶段的角度、频率与推进节拍。
 *
 * 这里不直接输出电压，也不做电流闭环。
 * 它只产生“开环电角度”和“开环频率”，供 app_foc 决定如何驱动电机。
 */

typedef struct
{
  uint32_t align_ticks; /* 对齐阶段累计 tick，用于判断对齐时间是否到。 */
  float mech_freq_hz;   /* 当前开环机械频率，单位 Hz。 */
  float theta_e_rad;    /* 当前开环电角度，单位 rad，范围 [0, 2pi)。 */
} FOC_OpenLoopData_t;

/* 初始化/复位开环轨迹状态。 */
void FOC_OpenLoopInit(FOC_OpenLoopData_t *openloop);
void FOC_OpenLoopResetAlign(FOC_OpenLoopData_t *openloop);
void FOC_OpenLoopResetRun(FOC_OpenLoopData_t *openloop);

/* 推进一拍开环频率斜坡和电角度。 */
void FOC_OpenLoopAdvance(FOC_OpenLoopData_t *openloop);

/* 获取当前开环电角速度 / 机械频率 / 电角度。 */
float FOC_OpenLoopGetElecSpeedRadPerSec(const FOC_OpenLoopData_t *openloop);
float FOC_OpenLoopGetFreqHz(const FOC_OpenLoopData_t *openloop);
float FOC_OpenLoopGetThetaErad(const FOC_OpenLoopData_t *openloop);
#ifdef __cplusplus
}

#endif
#endif
