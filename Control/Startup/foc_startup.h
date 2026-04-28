#ifndef FOC_STARTUP_H
#define FOC_STARTUP_H
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
} FOC_Startup_t;

/* 初始化/复位开环轨迹状态。 */
void FOC_Startup_Init(FOC_Startup_t *startup);
void FOC_Startup_ResetAlign(FOC_Startup_t *startup);
void FOC_Startup_ResetRun(FOC_Startup_t *startup);

/* 推进一拍开环频率斜坡和电角度。 */
void FOC_Startup_Update(FOC_Startup_t *startup);

/* 获取当前开环电角速度 / 机械频率 / 电角度。 */
float FOC_Startup_GetElecSpeedRadPerSec(const FOC_Startup_t *startup);
float FOC_Startup_GetFreqHz(const FOC_Startup_t *startup);
float FOC_Startup_GetThetaErad(const FOC_Startup_t *startup);
#ifdef __cplusplus
}

#endif
#endif
