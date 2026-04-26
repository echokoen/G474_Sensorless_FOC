#ifndef __FOC_CURRENT_LOOP_H__
#define __FOC_CURRENT_LOOP_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "PMSM_Control_Core/PI_Controller.h"
#include "PMSM_Control_Core/Clarke_Park.h"
#include "foc_sampling.h"
#include <stdint.h>

/* dq 电流环模块接口。
 *
 * 提供电流采样测量、dq 电流控制以及调试量读取接口。
 *
 * 真实控制链：
 * 三相电流 -> Clarke -> Park -> id/iq 测量
 * -> d/q PI
 * -> 电压极性映射
 * -> dq 矢量限幅
 * -> InvPark
 * -> valpha/vbeta
 */

typedef struct
{
  /* d/q 两个 PI 控制器，积分状态保存在这里。 */
  PIController_t id_pi;
  PIController_t iq_pi;
  /* 当前电流参考与测量值。 */
  float id_ref_a;
  float iq_ref_a;
  float id_meas_a;
  float iq_meas_a;
  /* PI、前馈和限幅后的 dq 电压命令。 */
  float vd_pi_v;
  float vq_pi_v;
  float vd_ff_v;
  float vq_ff_v;
  float vd_ff_raw_v;
  float vq_ff_raw_v;
  float bemf_ff_blend;
  float decouple_ff_blend;
  uint32_t decouple_ff_delay_ticks;
  uint8_t ff_enable;
  float vd_cmd_v;
  float vq_cmd_v;
  /* 反 Park 后送给 SVPWM 的 alpha/beta 电压命令。 */
  float valpha_cmd_v;
  float vbeta_cmd_v;
  /* 当前用于 Park/InvPark 的控制角。 */
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

void FOC_CurrentLoopRunIqOnly(FOC_CurrentLoopData_t *loop,
                              const FOC_SamplingData_t *sampling,
                              float theta_rad,
                              float iq_ref_a,
                              float *valpha,
                              float *vbeta);

void FOC_CurrentLoopRunDSoft(FOC_CurrentLoopData_t *loop,
                             const FOC_SamplingData_t *sampling,
                             float theta_rad,
                             float id_ref_a,
                             float iq_ref_a,
                             float d_axis_enable_k,
                             float vd_cmd_limit_v,
                             float *valpha,
                             float *vbeta);

void FOC_CurrentLoopRunDSoftFeedForward(FOC_CurrentLoopData_t *loop,
                                        const FOC_SamplingData_t *sampling,
                                        float theta_rad,
                                        float id_ref_a,
                                        float iq_ref_a,
                                        float d_axis_enable_k,
                                        float vd_cmd_limit_v,
                                        float we_rad_s,
                                        float vbus_v,
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
