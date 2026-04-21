#include "foc_current_loop.h"
#include "foc_config.h"
#include <math.h>

/* dq 电流环实现。
 *
 * 负责三相电流到 dq 的变换、电流 PI、限幅以及反 Park 变换。
 * 启动轨迹、观测器接管和故障停机不在本模块内处理。
 */

static void foc_current_loop_limit_vector(float *x, float *y, float limit)
{
  if ((x == 0) || (y == 0) || (limit <= 0.0f))
  {
    return;
  }
  {
    const float mag_sq = (*x * *x) + (*y * *y);
    const float limit_sq = limit * limit;

    if (mag_sq > limit_sq)
    {
      const float scale = limit / sqrtf(mag_sq);
      *x *= scale;
      *y *= scale;
    }
  }
}

void FOC_CurrentLoopInit(FOC_CurrentLoopData_t *loop)
{
  if (loop == 0)
  {
    return;
  }

  PIController_Init(&loop->id_pi, FOC_ID_KP, FOC_ID_KI, -FOC_CURR_LOOP_V_LIMIT, FOC_CURR_LOOP_V_LIMIT);
  PIController_Init(&loop->iq_pi, FOC_IQ_KP, FOC_IQ_KI, -FOC_CURR_LOOP_V_LIMIT, FOC_CURR_LOOP_V_LIMIT);
  FOC_CurrentLoopReset(loop);
}

void FOC_CurrentLoopReset(FOC_CurrentLoopData_t *loop)
{
  if (loop == 0)
  {
    return;
  }

  PIController_Reset(&loop->id_pi);
  PIController_Reset(&loop->iq_pi);
  loop->id_ref_a = FOC_ID_REF_A;
  loop->iq_ref_a = FOC_IQ_REF_A;
  loop->id_meas_a = 0.0f;
  loop->iq_meas_a = 0.0f;
  loop->vd_cmd_v = 0.0f;
  loop->vq_cmd_v = 0.0f;
  loop->valpha_cmd_v = 0.0f;
  loop->vbeta_cmd_v = 0.0f;
  loop->theta_ctrl_rad = 0.0f;
}

void FOC_CurrentLoopMeasureDQ(FOC_CurrentLoopData_t *loop,
                              const FOC_SamplingData_t *sampling,
                              float theta_rad,
                              ClarkePark_AlphaBeta_t *iab,
                              ClarkePark_DQ_t *idq)
{
  ClarkePark_AlphaBeta_t local_ab = {0.0f, 0.0f};
  ClarkePark_DQ_t local_dq = {0.0f, 0.0f};

  if ((loop == 0) || (sampling == 0))
  {
    return;
  }

  ClarkePark_Clarke(sampling->iu_a, sampling->iv_a, sampling->iw_a, &local_ab);
  ClarkePark_Park(&local_ab, theta_rad, &local_dq);
  loop->id_meas_a = local_dq.d;
  loop->iq_meas_a = local_dq.q;
  loop->theta_ctrl_rad = theta_rad;
  if (iab) *iab = local_ab;
  if (idq) *idq = local_dq;
}

void FOC_CurrentLoopRunOpenLoopCurrent(FOC_CurrentLoopData_t *loop,
                                       const FOC_SamplingData_t *sampling,
                                       float theta_rad,
                                       float *valpha,
                                       float *vbeta)
{
  FOC_CurrentLoopRunWithRef(loop,
                            sampling,
                            theta_rad,
                            FOC_ID_REF_A,
                            FOC_IQ_REF_A,
                            valpha,
                            vbeta);
}

void FOC_CurrentLoopRunWithRef(FOC_CurrentLoopData_t *loop,
                               const FOC_SamplingData_t *sampling,
                               float theta_rad,
                               float id_ref_a,
                               float iq_ref_a,
                               float *valpha,
                               float *vbeta)
{
  ClarkePark_AlphaBeta_t vab = {0.0f, 0.0f};
  ClarkePark_DQ_t idq = {0.0f, 0.0f};
  ClarkePark_DQ_t vdq = {0.0f, 0.0f};

  if ((loop == 0) || (sampling == 0))
  {
    return;
  }

  FOC_CurrentLoopMeasureDQ(loop, sampling, theta_rad, 0, &idq);
  loop->id_ref_a = id_ref_a;
  loop->iq_ref_a = iq_ref_a;
  vdq.d = FOC_VD_CMD_SIGN * PIController_Run(&loop->id_pi, loop->id_ref_a, idq.d, FOC_TS_SEC);
  vdq.q = FOC_VQ_CMD_SIGN * PIController_Run(&loop->iq_pi, loop->iq_ref_a, idq.q, FOC_TS_SEC);
  foc_current_loop_limit_vector(&vdq.d, &vdq.q, FOC_DQ_VOLT_LIMIT);
  ClarkePark_InvPark(&vdq, theta_rad, &vab);
  loop->vd_cmd_v = vdq.d;
  loop->vq_cmd_v = vdq.q;
  loop->valpha_cmd_v = vab.alpha;
  loop->vbeta_cmd_v = vab.beta;
  if (valpha) *valpha = loop->valpha_cmd_v;
  if (vbeta) *vbeta = loop->vbeta_cmd_v;
}

void FOC_CurrentLoopRunOpenLoopVoltTest(FOC_CurrentLoopData_t *loop,
                                        const FOC_SamplingData_t *sampling,
                                        float theta_rad,
                                        float *valpha,
                                        float *vbeta)
{
  ClarkePark_AlphaBeta_t vab = {0.0f, 0.0f};
  ClarkePark_DQ_t vdq = {FOC_DQ_TEST_VD_V, FOC_DQ_TEST_VQ_V};

  if ((loop == 0) || (sampling == 0))
  {
    return;
  }

  FOC_CurrentLoopMeasureDQ(loop, sampling, theta_rad, 0, 0);
  loop->id_ref_a = 0.0f;
  loop->iq_ref_a = 0.0f;
  foc_current_loop_limit_vector(&vdq.d, &vdq.q, FOC_DQ_VOLT_LIMIT);
  ClarkePark_InvPark(&vdq, theta_rad, &vab);
  loop->vd_cmd_v = vdq.d;
  loop->vq_cmd_v = vdq.q;
  loop->valpha_cmd_v = vab.alpha;
  loop->vbeta_cmd_v = vab.beta;
  if (valpha) *valpha = loop->valpha_cmd_v;
  if (vbeta) *vbeta = loop->vbeta_cmd_v;
}

void FOC_CurrentLoopRunOpenLoopVf(FOC_CurrentLoopData_t *loop,
                                  const FOC_SamplingData_t *sampling,
                                  float theta_rad,
                                  float *valpha,
                                  float *vbeta)
{
  if ((loop == 0) || (sampling == 0))
  {
    return;
  }

  FOC_CurrentLoopMeasureDQ(loop, sampling, theta_rad, 0, 0);
  loop->id_ref_a = FOC_ID_REF_A;
  loop->iq_ref_a = FOC_IQ_REF_A;
  loop->vd_cmd_v = FOC_OPENLOOP_VDQ_V;
  loop->vq_cmd_v = 0.0f;
  loop->valpha_cmd_v = FOC_OPENLOOP_VDQ_V * cosf(theta_rad);
  loop->vbeta_cmd_v = FOC_OPENLOOP_VDQ_V * sinf(theta_rad);
  if (valpha) *valpha = loop->valpha_cmd_v;
  if (vbeta) *vbeta = loop->vbeta_cmd_v;
}

void FOC_CurrentLoopGetDebug(const FOC_CurrentLoopData_t *loop,
                             float *id_ref,
                             float *iq_ref,
                             float *id_meas,
                             float *iq_meas,
                             float *vd_cmd,
                             float *vq_cmd,
                             float *valpha_cmd,
                             float *vbeta_cmd,
                             float *theta_ctrl)
{
  if (loop == 0)
  {
    return;
  }

  if (id_ref) *id_ref = loop->id_ref_a;
  if (iq_ref) *iq_ref = loop->iq_ref_a;
  if (id_meas) *id_meas = loop->id_meas_a;
  if (iq_meas) *iq_meas = loop->iq_meas_a;
  if (vd_cmd) *vd_cmd = loop->vd_cmd_v;
  if (vq_cmd) *vq_cmd = loop->vq_cmd_v;
  if (valpha_cmd) *valpha_cmd = loop->valpha_cmd_v;
  if (vbeta_cmd) *vbeta_cmd = loop->vbeta_cmd_v;
  if (theta_ctrl) *theta_ctrl = loop->theta_ctrl_rad;
}
