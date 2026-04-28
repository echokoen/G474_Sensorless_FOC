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
  /* dq 电压矢量限幅。
   * 注意这里不是 d/q 分别限幅，而是限制 sqrt(vd^2+vq^2)。
   * 这样可以保持电压矢量方向不变，只缩小幅值。
   */
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

static void foc_current_loop_set_pi_limit(PIController_t *pi, float limit)
{
  if ((pi == 0) || (limit <= 0.0f))
  {
    return;
  }

  pi->integral_min = -limit;
  pi->integral_max = limit;
  pi->out_min = -limit;
  pi->out_max = limit;
}


static float foc_current_loop_absf(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static float foc_current_loop_clampf(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }
  if (value < min_value)
  {
    return min_value;
  }
  return value;
}

static float foc_current_loop_ramp_step(float ramp_ms)
{
  if (ramp_ms <= 0.0f)
  {
    return 1.0f;
  }

  return (FOC_TS_SEC * 1000.0f) / ramp_ms;
}

static void foc_current_loop_clear_feedforward(FOC_CurrentLoopData_t *loop)
{
  if (loop == 0)
  {
    return;
  }

  loop->vd_ff_v = 0.0f;
  loop->vq_ff_v = 0.0f;
  loop->vd_ff_raw_v = 0.0f;
  loop->vq_ff_raw_v = 0.0f;
  loop->bemf_ff_blend = 0.0f;
  loop->decouple_ff_blend = 0.0f;
  loop->decouple_ff_delay_ticks = 0u;
  loop->ff_enable = 0u;
}

static void foc_current_loop_update_feedforward(FOC_CurrentLoopData_t *loop,
                                                float id_ref_a,
                                                float iq_ref_a,
                                                float we_rad_s,
                                                float vbus_v)
{
  float we_used = we_rad_s;
  float v_limit = FOC_DQ_VOLT_LIMIT;
  float vd_ff_limit = 0.0f;
  float vq_ff_limit = 0.0f;
  float vd_dec_raw = 0.0f;
  float vq_dec_raw = 0.0f;
  float vq_bemf_raw = 0.0f;
  float vd_ff = 0.0f;
  float vq_ff = 0.0f;

  if (loop == 0)
  {
    return;
  }

#if (FOC_FF_ENABLE == 0u)
  foc_current_loop_clear_feedforward(loop);
  return;
#else
  if ((vbus_v < FOC_FF_MIN_VBUS_V) ||
      (foc_current_loop_absf(we_used) < FOC_FF_MIN_ELEC_SPEED_RAD_S))
  {
    foc_current_loop_clear_feedforward(loop);
    return;
  }

  we_used = foc_current_loop_clampf(we_used,
                                    -FOC_FF_SPEED_LIMIT_RAD_S,
                                    FOC_FF_SPEED_LIMIT_RAD_S);

  /* 可用 dq 电压按母线估算一次，再和工程固定限幅取小值。
   * 这样即使母线电压下跌，前馈项也不会仍按满电压强行补偿。
   */
  {
    const float vbus_limit = vbus_v * 0.577350269f;
    if ((vbus_limit > 0.0f) && (vbus_limit < v_limit))
    {
      v_limit = vbus_limit;
    }
  }

  vd_ff_limit = FOC_FF_VD_LIMIT_RATIO * v_limit;
  vq_ff_limit = FOC_FF_VQ_LIMIT_RATIO * v_limit;

#if (FOC_BEMF_FF_ENABLE != 0u)
  loop->bemf_ff_blend += foc_current_loop_ramp_step(FOC_BEMF_FF_RAMP_MS);
  if (loop->bemf_ff_blend > 1.0f)
  {
    loop->bemf_ff_blend = 1.0f;
  }
#else
  loop->bemf_ff_blend = 0.0f;
#endif

#if (FOC_DECOUPLE_FF_ENABLE != 0u)
  if (FOC_DECOUPLE_FF_GAIN_TARGET > 0.0f)
  {
    const uint32_t delay_ticks = (uint32_t)((FOC_DECOUPLE_FF_DELAY_MS * 0.001f) / FOC_TS_SEC);

    if (loop->decouple_ff_delay_ticks < delay_ticks)
    {
      loop->decouple_ff_delay_ticks++;
      loop->decouple_ff_blend = 0.0f;
    }
    else
    {
      loop->decouple_ff_blend += foc_current_loop_ramp_step(FOC_DECOUPLE_FF_RAMP_MS);
      if (loop->decouple_ff_blend > 1.0f)
      {
        loop->decouple_ff_blend = 1.0f;
      }
    }
  }
  else
  {
    loop->decouple_ff_delay_ticks = 0u;
    loop->decouple_ff_blend = 0.0f;
  }
#else
  loop->decouple_ff_delay_ticks = 0u;
  loop->decouple_ff_blend = 0.0f;
#endif

  /* 这里先按“逻辑 dq 电压方程”计算，再乘 FOC_VD/VQ_CMD_SIGN 映射到本工程实际输出极性。
   * 如果前馈打开后现象变差，优先调 FOC_BEMF_FF_SIGN 或 FOC_DECOUPLE_FF_SIGN。
   */
  vd_dec_raw  = FOC_DECOUPLE_FF_SIGN * FOC_VD_CMD_SIGN * (-we_used * FOC_LQ_H * iq_ref_a);
  vq_dec_raw  = FOC_DECOUPLE_FF_SIGN * FOC_VQ_CMD_SIGN * ( we_used * FOC_LD_H * id_ref_a);
  vq_bemf_raw = FOC_BEMF_FF_SIGN     * FOC_VQ_CMD_SIGN * ( we_used * FOC_FLUX_WB);

  loop->vd_ff_raw_v = vd_dec_raw;
  loop->vq_ff_raw_v = vq_dec_raw + vq_bemf_raw;

  vd_ff = loop->decouple_ff_blend * FOC_DECOUPLE_FF_GAIN_TARGET * vd_dec_raw;
  vq_ff = (loop->decouple_ff_blend * FOC_DECOUPLE_FF_GAIN_TARGET * vq_dec_raw) +
          (loop->bemf_ff_blend     * FOC_BEMF_FF_GAIN_TARGET     * vq_bemf_raw);

  loop->vd_ff_v = foc_current_loop_clampf(vd_ff, -vd_ff_limit, vd_ff_limit);
  loop->vq_ff_v = foc_current_loop_clampf(vq_ff, -vq_ff_limit, vq_ff_limit);
  loop->ff_enable = 1u;
#endif
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
  loop->vd_pi_v = 0.0f;
  loop->vq_pi_v = 0.0f;
  foc_current_loop_clear_feedforward(loop);
  loop->vd_cmd_v = 0.0f;
  loop->vq_cmd_v = 0.0f;
  loop->valpha_cmd_v = 0.0f;
  loop->vbeta_cmd_v = 0.0f;
  loop->theta_ctrl_rad = 0.0f;
}

void FOC_CurrentLoopMeasureDQ(FOC_CurrentLoopData_t *loop,
                              const FOC_CurrentSense_t *current_sense,
                              float theta_rad,
                              ClarkePark_AlphaBeta_t *iab,
                              ClarkePark_DQ_t *idq)
{
  /* 单独测量 id/iq，便于：
   * - 电流环正常运行前做方向测试；
   * - 开环/电压测试模式下仍然能打印 id/iq；
   * - observer 调试时确认采样和坐标变换是否一致。
   */
  ClarkePark_AlphaBeta_t local_ab = {0.0f, 0.0f};
  ClarkePark_DQ_t local_dq = {0.0f, 0.0f};

  if ((loop == 0) || (current_sense == 0))
  {
    return;
  }

  ClarkePark_Clarke(current_sense->iu_a, current_sense->iv_a, current_sense->iw_a, &local_ab);
  ClarkePark_Park(&local_ab, theta_rad, &local_dq);
  loop->id_meas_a = local_dq.d;
  loop->iq_meas_a = local_dq.q;
  loop->theta_ctrl_rad = theta_rad;
  if (iab) *iab = local_ab;
  if (idq) *idq = local_dq;
}

void FOC_CurrentLoopRunOpenLoopCurrent(FOC_CurrentLoopData_t *loop,
                                       const FOC_CurrentSense_t *current_sense,
                                       float theta_rad,
                                       float *valpha,
                                       float *vbeta)
{
  FOC_CurrentLoopRunWithRef(loop,
                            current_sense,
                            theta_rad,
                            FOC_ID_REF_A,
                            FOC_IQ_REF_A,
                            valpha,
                            vbeta);
}

void FOC_CurrentLoopRunWithRef(FOC_CurrentLoopData_t *loop,
                               const FOC_CurrentSense_t *current_sense,
                               float theta_rad,
                               float id_ref_a,
                               float iq_ref_a,
                               float *valpha,
                               float *vbeta)
{
  ClarkePark_AlphaBeta_t vab = {0.0f, 0.0f};
  ClarkePark_DQ_t idq = {0.0f, 0.0f};
  ClarkePark_DQ_t vdq = {0.0f, 0.0f};

  if ((loop == 0) || (current_sense == 0))
  {
    return;
  }

  /* 1. 三相电流变换到当前控制角下的 dq 电流。 */
  FOC_CurrentLoopMeasureDQ(loop, current_sense, theta_rad, 0, &idq);
  loop->id_ref_a = id_ref_a;
  loop->iq_ref_a = iq_ref_a;

  /* 2. d/q 两轴独立 PI。
   * FOC_VD_CMD_SIGN / FOC_VQ_CMD_SIGN 用于适配实际硬件极性。
   */
  foc_current_loop_set_pi_limit(&loop->id_pi, FOC_CURR_LOOP_V_LIMIT);
  foc_current_loop_set_pi_limit(&loop->iq_pi, FOC_CURR_LOOP_V_LIMIT);
  vdq.d = FOC_VD_CMD_SIGN * PIController_Run(&loop->id_pi, loop->id_ref_a, idq.d, FOC_TS_SEC);
  loop->vd_pi_v = vdq.d;
  vdq.q = FOC_VQ_CMD_SIGN * PIController_Run(&loop->iq_pi, loop->iq_ref_a, idq.q, FOC_TS_SEC);
  loop->vq_pi_v = vdq.q;
  foc_current_loop_clear_feedforward(loop);

  /* 3. dq 电压矢量总幅值限幅，避免超出母线可输出范围。 */
  foc_current_loop_limit_vector(&vdq.d, &vdq.q, FOC_DQ_VOLT_LIMIT);

  /* 4. 转回 alpha/beta，交给 PWM 模块。 */
  ClarkePark_InvPark(&vdq, theta_rad, &vab);
  loop->vd_cmd_v = vdq.d;
  loop->vq_cmd_v = vdq.q;
  loop->valpha_cmd_v = vab.alpha;
  loop->vbeta_cmd_v = vab.beta;
  if (valpha) *valpha = loop->valpha_cmd_v;
  if (vbeta) *vbeta = loop->vbeta_cmd_v;
}

void FOC_CurrentLoopRunIqOnly(FOC_CurrentLoopData_t *loop,
                              const FOC_CurrentSense_t *current_sense,
                              float theta_rad,
                              float iq_ref_a,
                              float *valpha,
                              float *vbeta)
{
  ClarkePark_AlphaBeta_t vab = {0.0f, 0.0f};
  ClarkePark_DQ_t idq = {0.0f, 0.0f};
  ClarkePark_DQ_t vdq = {0.0f, 0.0f};

  if ((loop == 0) || (current_sense == 0))
  {
    return;
  }

  FOC_CurrentLoopMeasureDQ(loop, current_sense, theta_rad, 0, &idq);
  loop->id_ref_a = FOC_ID_REF_A;
  loop->iq_ref_a = iq_ref_a;
  PIController_Reset(&loop->id_pi);
  loop->vd_pi_v = 0.0f;
  vdq.d = 0.0f;
  vdq.q = FOC_VQ_CMD_SIGN * PIController_Run(&loop->iq_pi, loop->iq_ref_a, idq.q, FOC_TS_SEC);
  loop->vq_pi_v = vdq.q;
  foc_current_loop_clear_feedforward(loop);
  foc_current_loop_limit_vector(&vdq.d, &vdq.q, FOC_DQ_VOLT_LIMIT);
  ClarkePark_InvPark(&vdq, theta_rad, &vab);
  loop->vd_cmd_v = 0.0f;
  loop->vq_cmd_v = vdq.q;
  loop->valpha_cmd_v = vab.alpha;
  loop->vbeta_cmd_v = vab.beta;
  if (valpha) *valpha = loop->valpha_cmd_v;
  if (vbeta) *vbeta = loop->vbeta_cmd_v;
}

void FOC_CurrentLoopRunDSoft(FOC_CurrentLoopData_t *loop,
                             const FOC_CurrentSense_t *current_sense,
                             float theta_rad,
                             float id_ref_a,
                             float iq_ref_a,
                             float d_axis_enable_k,
                             float vd_cmd_limit_v,
                             float *valpha,
                             float *vbeta)
{
  ClarkePark_AlphaBeta_t vab = {0.0f, 0.0f};
  ClarkePark_DQ_t idq = {0.0f, 0.0f};
  ClarkePark_DQ_t vdq = {0.0f, 0.0f};
  float vd_pi = 0.0f;
  float k = d_axis_enable_k;

  if ((loop == 0) || (current_sense == 0))
  {
    return;
  }

  if (k < 0.0f)
  {
    k = 0.0f;
  }
  else if (k > 1.0f)
  {
    k = 1.0f;
  }

  FOC_CurrentLoopMeasureDQ(loop, current_sense, theta_rad, 0, &idq);
  loop->id_ref_a = id_ref_a;
  loop->iq_ref_a = iq_ref_a;
  foc_current_loop_set_pi_limit(&loop->id_pi, vd_cmd_limit_v);
  foc_current_loop_set_pi_limit(&loop->iq_pi, FOC_CURR_LOOP_V_LIMIT);

  if (k <= 0.0f)
  {
    /* k=0 时保持当前稳定的 Iq-only 行为，冻结 d 轴积分。 */
    PIController_Reset(&loop->id_pi);
    vd_pi = 0.0f;
  }
  else
  {
    vd_pi = FOC_VD_CMD_SIGN * PIController_Run(&loop->id_pi,
                                               loop->id_ref_a,
                                               idq.d,
                                               FOC_TS_SEC);
  }

  loop->vd_pi_v = vd_pi;

  vdq.d = vd_pi * k;
  if (vdq.d > vd_cmd_limit_v)
  {
    vdq.d = vd_cmd_limit_v;
  }
  else if (vdq.d < -vd_cmd_limit_v)
  {
    vdq.d = -vd_cmd_limit_v;
  }

  vdq.q = FOC_VQ_CMD_SIGN * PIController_Run(&loop->iq_pi,
                                             loop->iq_ref_a,
                                             idq.q,
                                             FOC_TS_SEC);
  loop->vq_pi_v = vdq.q;
  foc_current_loop_clear_feedforward(loop);

  foc_current_loop_limit_vector(&vdq.d, &vdq.q, FOC_DQ_VOLT_LIMIT);
  ClarkePark_InvPark(&vdq, theta_rad, &vab);

  loop->vd_cmd_v = vdq.d;
  loop->vq_cmd_v = vdq.q;
  loop->valpha_cmd_v = vab.alpha;
  loop->vbeta_cmd_v = vab.beta;
  if (valpha) *valpha = loop->valpha_cmd_v;
  if (vbeta) *vbeta = loop->vbeta_cmd_v;
}

void FOC_CurrentLoopRunDSoftFeedForward(FOC_CurrentLoopData_t *loop,
                                        const FOC_CurrentSense_t *current_sense,
                                        float theta_rad,
                                        float id_ref_a,
                                        float iq_ref_a,
                                        float d_axis_enable_k,
                                        float vd_cmd_limit_v,
                                        float we_rad_s,
                                        float vbus_v,
                                        float *valpha,
                                        float *vbeta)
{
  ClarkePark_AlphaBeta_t vab = {0.0f, 0.0f};
  ClarkePark_DQ_t idq = {0.0f, 0.0f};
  ClarkePark_DQ_t vdq = {0.0f, 0.0f};
  float vd_pi = 0.0f;
  float k = d_axis_enable_k;

  if ((loop == 0) || (current_sense == 0))
  {
    return;
  }

  if (k < 0.0f)
  {
    k = 0.0f;
  }
  else if (k > 1.0f)
  {
    k = 1.0f;
  }

  FOC_CurrentLoopMeasureDQ(loop, current_sense, theta_rad, 0, &idq);
  loop->id_ref_a = id_ref_a;
  loop->iq_ref_a = iq_ref_a;
  foc_current_loop_set_pi_limit(&loop->id_pi, vd_cmd_limit_v);
  foc_current_loop_set_pi_limit(&loop->iq_pi, FOC_CURR_LOOP_V_LIMIT);

  if (k <= 0.0f)
  {
    PIController_Reset(&loop->id_pi);
    vd_pi = 0.0f;
  }
  else
  {
    vd_pi = FOC_VD_CMD_SIGN * PIController_Run(&loop->id_pi,
                                               loop->id_ref_a,
                                               idq.d,
                                               FOC_TS_SEC);
  }

  loop->vd_pi_v = vd_pi;

  vdq.d = vd_pi * k;
  if (vdq.d > vd_cmd_limit_v)
  {
    vdq.d = vd_cmd_limit_v;
  }
  else if (vdq.d < -vd_cmd_limit_v)
  {
    vdq.d = -vd_cmd_limit_v;
  }

  vdq.q = FOC_VQ_CMD_SIGN * PIController_Run(&loop->iq_pi,
                                             loop->iq_ref_a,
                                             idq.q,
                                             FOC_TS_SEC);
  loop->vq_pi_v = vdq.q;

  /* RUN 阶段前馈：PI 输出后叠加，随后统一做 dq 电压矢量限幅。 */
  foc_current_loop_update_feedforward(loop,
                                      loop->id_ref_a,
                                      loop->iq_ref_a,
                                      we_rad_s,
                                      vbus_v);

  vdq.d += loop->vd_ff_v;
  vdq.q += loop->vq_ff_v;

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
                                        const FOC_CurrentSense_t *current_sense,
                                        float theta_rad,
                                        float *valpha,
                                        float *vbeta)
{
  ClarkePark_AlphaBeta_t vab = {0.0f, 0.0f};
  ClarkePark_DQ_t vdq = {FOC_DQ_TEST_VD_V, FOC_DQ_TEST_VQ_V};

  if ((loop == 0) || (current_sense == 0))
  {
    return;
  }

  FOC_CurrentLoopMeasureDQ(loop, current_sense, theta_rad, 0, 0);
  loop->id_ref_a = 0.0f;
  loop->iq_ref_a = 0.0f;
  loop->vd_pi_v = vdq.d;
  loop->vq_pi_v = vdq.q;
  foc_current_loop_clear_feedforward(loop);
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
                                  const FOC_CurrentSense_t *current_sense,
                                  float theta_rad,
                                  float *valpha,
                                  float *vbeta)
{
  if ((loop == 0) || (current_sense == 0))
  {
    return;
  }

  FOC_CurrentLoopMeasureDQ(loop, current_sense, theta_rad, 0, 0);
  loop->id_ref_a = FOC_ID_REF_A;
  loop->iq_ref_a = FOC_IQ_REF_A;
  loop->vd_pi_v = FOC_OPENLOOP_VDQ_V;
  loop->vq_pi_v = 0.0f;
  foc_current_loop_clear_feedforward(loop);
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
