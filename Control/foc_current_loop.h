#ifndef __FOC_CURRENT_LOOP_H__
#define __FOC_CURRENT_LOOP_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "PMSM_Control_Core/PI_Controller.h"
#include "PMSM_Control_Core/Clarke_Park.h"
#include "foc_sampling.h"

/* dq 电流环模块接口。
 *
 * 提供电流采样测量、dq 电流控制以及调试量读取接口。
 */

typedef struct
{
  PIController_t id_pi;
  PIController_t iq_pi;
  float id_ref_a;
  float iq_ref_a;
  float id_meas_a;
  float iq_meas_a;
  float vd_cmd_v;
  float vq_cmd_v;
  float valpha_cmd_v;
  float vbeta_cmd_v;
  float theta_ctrl_rad;
} FOC_CurrentLoopData_t;
void FOC_CurrentLoopInit(FOC_CurrentLoopData_t *loop);
void FOC_CurrentLoopReset(FOC_CurrentLoopData_t *loop);

void FOC_CurrentLoopMeasureDQ(FOC_CurrentLoopData_t *loop,
                              const FOC_SamplingData_t *sampling,
                              float theta_rad,
                              ClarkePark_AlphaBeta_t *iab,
                              ClarkePark_DQ_t *idq);

void FOC_CurrentLoopRunOpenLoopCurrent(FOC_CurrentLoopData_t *loop,
                                       const FOC_SamplingData_t *sampling,
                                       float theta_rad,
                                       float *valpha,
                                       float *vbeta);

void FOC_CurrentLoopRunWithRef(FOC_CurrentLoopData_t *loop,
                               const FOC_SamplingData_t *sampling,
                               float theta_rad,
                               float id_ref_a,
                               float iq_ref_a,
                               float *valpha,
                               float *vbeta);

void FOC_CurrentLoopRunOpenLoopVoltTest(FOC_CurrentLoopData_t *loop,
                                        const FOC_SamplingData_t *sampling,
                                        float theta_rad,
                                        float *valpha,
                                        float *vbeta);

void FOC_CurrentLoopRunOpenLoopVf(FOC_CurrentLoopData_t *loop,
                                  const FOC_SamplingData_t *sampling,
                                  float theta_rad,
                                  float *valpha,
                                  float *vbeta);

void FOC_CurrentLoopGetDebug(const FOC_CurrentLoopData_t *loop,
                             float *id_ref,
                             float *iq_ref,
                             float *id_meas,
                             float *iq_meas,
                             float *vd_cmd,
                             float *vq_cmd,
                             float *valpha_cmd,
                             float *vbeta_cmd,
                             float *theta_ctrl);
#ifdef __cplusplus
}

#endif
#endif
