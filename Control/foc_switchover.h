#ifndef __FOC_SWITCHOVER_H__
#define __FOC_SWITCHOVER_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "foc_observer.h"
#include "foc_openloop.h"

/* 观测器接管模块接口。
 *
 * 用于管理开环角与观测器角之间的切换状态。
 */

typedef enum
{
  FOC_SW_STATE_OPENLOOP = 0,
  FOC_SW_STATE_BLEND,
  FOC_SW_STATE_OBSERVER
} FOC_SwitchoverState_t;

typedef struct
{
  uint8_t observer_ready;
  uint32_t hold_ticks;
  uint32_t blend_ticks;
  float blend_k;
  FOC_SwitchoverState_t state;
} FOC_SwitchoverData_t;
void FOC_SwitchoverInit(FOC_SwitchoverData_t *sw);
void FOC_SwitchoverReset(FOC_SwitchoverData_t *sw);

void FOC_SwitchoverUpdate(FOC_SwitchoverData_t *sw,
                          const FOC_OpenLoopData_t *openloop,
                          const FOC_ObserverData_t *observer);

float FOC_SwitchoverSelectTheta(const FOC_SwitchoverData_t *sw,
                                const FOC_OpenLoopData_t *openloop,
                                const FOC_ObserverData_t *observer);

void FOC_SwitchoverGetDebug(const FOC_SwitchoverData_t *sw,
                            const FOC_OpenLoopData_t *openloop,
                            const FOC_ObserverData_t *observer,
                            float theta_ctrl,
                            uint8_t *sw_state,
                            uint8_t *observer_ready,
                            float *blend_k,
                            float *theta_open,
                            float *theta_obs,
                            float *theta_ctrl_out,
                            float *angle_err_deg,
                            float *open_speed_rad_s,
                            float *obs_speed_rad_s);
#ifdef __cplusplus
}

#endif
#endif
