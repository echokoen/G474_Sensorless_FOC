#ifndef __FOC_OBSERVER_H__
#define __FOC_OBSERVER_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "PMSM_Control_Core/FluxObserver_PLL.h"
#include "foc_sampling.h"

/* 观测器封装接口。
 *
 * 该模块位于 Control 层，下面接 FluxObserver_PLL 内核，
 * 上面给 app_foc / switchover 提供“可直接使用”的角度和速度。
 *
 * 注意：
 * - 这里不实现磁链积分和 PLL 公式，真正算法在 FluxObserver_PLL.c；
 * - 这里负责把三相采样结果转换成 alpha/beta 电流；
 * - 这里负责加上工程里的角度补偿 FOC_OBS_THETA_COMP_RAD；
 * - observer 是否可以接管开环角，由 foc_switchover.c 判断。
 */

typedef struct
{
  struct FluxObserver_PLL_t flux_obs; /* 观测器内核的输入输出与内部接口结构。 */
  float theta_rad;                    /* 补偿后的 observer 电角度，单位 rad。 */
  float speed_rad_s;                  /* observer 输出的电角速度，单位 rad/s。 */
} FOC_ObserverData_t;

/* 初始化/复位 observer 封装层与 FluxObserver_PLL 内核状态。 */
void FOC_ObserverInit(FOC_ObserverData_t *observer);
void FOC_ObserverReset(FOC_ObserverData_t *observer);

/* 执行一拍 observer 更新。
 *
 * 输入：
 * - sampling：当前三相电流反馈；
 * - valpha/vbeta：本拍实际送入 PWM 的 alpha/beta 电压；
 */
void FOC_ObserverUpdate(FOC_ObserverData_t *observer,
                        const FOC_SamplingData_t *sampling,
                        float valpha,
                        float vbeta);
float FOC_ObserverGetFluxThetaRad(const FOC_ObserverData_t *observer);
float FOC_ObserverGetThetaRad(const FOC_ObserverData_t *observer);
float FOC_ObserverGetSpeedRadPerSec(const FOC_ObserverData_t *observer);

/* Compatibility stubs: switchover now owns angle-error and locked state. */
float FOC_ObserverGetAngleErrDeg(const FOC_ObserverData_t *observer);
uint8_t FOC_ObserverIsLocked(const FOC_ObserverData_t *observer);

/* 读取 observer 最近一拍使用的输入，主要给调试打印和上位机观察。 */
void FOC_ObserverGetInputs(const FOC_ObserverData_t *observer,
                           float *valpha,
                           float *vbeta,
                           float *ialpha,
                           float *ibeta);
#ifdef __cplusplus
}

#endif
#endif
