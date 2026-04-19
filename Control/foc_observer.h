#ifndef __FOC_OBSERVER_H__
#define __FOC_OBSERVER_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "PMSM_Control_Core/FluxObserver_PLL.h"
#include "foc_sampling.h"

/* 观测器封装接口。
 *
 * 该模块只负责调用观测器内核并对结果做轻量封装。
 */

typedef struct
{
  struct FluxObserver_PLL_t flux_obs;
  float theta_rad;
  float speed_rad_s;
  float angle_err_deg;
  uint8_t locked;
  uint32_t hold_ticks;
} FOC_ObserverData_t;
void FOC_ObserverInit(FOC_ObserverData_t *observer);
void FOC_ObserverReset(FOC_ObserverData_t *observer);

void FOC_ObserverUpdate(FOC_ObserverData_t *observer,
                        const FOC_SamplingData_t *sampling,
                        float valpha,
                        float vbeta,
                        float theta_open_rad,
                        float mech_freq_hz);
float FOC_ObserverGetFluxThetaRad(const FOC_ObserverData_t *observer);
float FOC_ObserverGetThetaRad(const FOC_ObserverData_t *observer);
float FOC_ObserverGetSpeedRadPerSec(const FOC_ObserverData_t *observer);
float FOC_ObserverGetAngleErrDeg(const FOC_ObserverData_t *observer);
uint8_t FOC_ObserverIsLocked(const FOC_ObserverData_t *observer);

void FOC_ObserverGetInputs(const FOC_ObserverData_t *observer,
                           float *valpha,
                           float *vbeta,
                           float *ialpha,
                           float *ibeta);
#ifdef __cplusplus
}

#endif
#endif
