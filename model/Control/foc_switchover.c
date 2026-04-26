#include "foc_switchover.h"
#include "foc_config.h"
#include "foc_math.h"
#include <math.h>

/* 观测器接管与过渡实现。
 *
 * 负责判断接管条件、执行混合过渡并输出最终控制角。
 *
 * 这层的核心目标不是“尽快接管”，而是“确认 observer 真的可信再接管”：
 * - 机械频率要达到门限；
 * - observer 角和开环角要接近；
 * - observer 速度和开环速度要接近；
 * - 条件还要连续保持一段时间。
 */
static uint8_t foc_switchover_condition(FOC_SwitchoverData_t *sw,
                                        const FOC_OpenLoopData_t *openloop,
                                        const FOC_ObserverData_t *observer)
{
  const float open_speed = FOC_OpenLoopGetElecSpeedRadPerSec(openloop);
  const float obs_speed = FOC_ObserverGetSpeedRadPerSec(observer);
  const float speed_err = fabsf(obs_speed - open_speed);
  float speed_limit = fabsf(open_speed) * FOC_SWITCHOVER_MAX_SPEED_ERR_RATIO;

  if (sw != 0)
  {
    sw->open_speed_rad_s = open_speed;
    sw->obs_speed_rad_s = obs_speed;
    sw->speed_err_rad_s = speed_err;
    sw->ready_now = 0u;
    sw->fail_reason = 0u;
  }

  /* 低速时开环电角速度本身较小，
   * 如果只按比例门限判断，速度误差窗口会过于苛刻，
   * 容易导致明明 observer 已经基本跟上，却迟迟不给接管。 */
  if (speed_limit < FOC_SWITCHOVER_MAX_SPEED_ERR_MIN_RAD_S)
  {
    speed_limit = FOC_SWITCHOVER_MAX_SPEED_ERR_MIN_RAD_S;
  }

  if (FOC_OpenLoopGetFreqHz(openloop) < FOC_SWITCHOVER_MIN_MECH_FREQ_HZ)
  {
    if (sw != 0) { sw->fail_reason |= 0x01u; }
    return 0u;
  }

  if (fabsf(FOC_ObserverGetAngleErrDeg(observer)) > FOC_SWITCHOVER_MAX_ANGLE_ERR_DEG)
  {
    if (sw != 0) { sw->fail_reason |= 0x02u; }
    return 0u;
  }

  /* 速度判定：
   * 只有当 observer 速度与开环速度足够接近时，
   * 才允许进入 switchover ready。
   * 这样可以避免“角度偶然对上，但速度其实没跟住”导致的误接管。 */
  if (speed_err > speed_limit)
  {
    if (sw != 0) { sw->fail_reason |= 0x04u; }
    return 0u;
  }

  if (sw != 0)
  {
    sw->ready_now = 1u;
  }

  return 1u;
}

void FOC_SwitchoverInit(FOC_SwitchoverData_t *sw)
{
  FOC_SwitchoverReset(sw);
}

void FOC_SwitchoverReset(FOC_SwitchoverData_t *sw)
{
  if (sw == 0)
  {
    return;
  }

  sw->observer_ready = 0u;
  sw->ready_now = 0u;
  sw->hold_ticks = 0u;
  sw->blend_ticks = 0u;
  sw->blend_k = 0.0f;
  sw->open_speed_rad_s = 0.0f;
  sw->obs_speed_rad_s = 0.0f;
  sw->speed_err_rad_s = 0.0f;
  sw->fail_reason = 0u;
  sw->last_blend_reset_reason = 0u;
  sw->blend_fail_ticks = 0u;
  sw->blend_reset_count = 0u;
  sw->last_blend_reset_angle_err_deg = 0.0f;
  sw->state = FOC_SW_STATE_OPENLOOP;
}

void FOC_SwitchoverUpdate(FOC_SwitchoverData_t *sw,
                          const FOC_OpenLoopData_t *openloop,
                          const FOC_ObserverData_t *observer)
{
#if (FOC_SWITCHOVER_ENABLE != 0u)

  const float inv_ctrl_period_ms = 1.0f / (FOC_TS_SEC * 1000.0f);
  const uint32_t hold_ticks = (uint32_t)(FOC_SWITCHOVER_HOLD_MS * inv_ctrl_period_ms);
  const uint32_t blend_ticks = (uint32_t)(FOC_SWITCHOVER_BLEND_MS * inv_ctrl_period_ms);
  uint8_t ready_now;

  if ((sw == 0) || (openloop == 0) || (observer == 0))
  {
    return;
  }

  ready_now = foc_switchover_condition(sw, openloop, observer);

  if (sw->state == FOC_SW_STATE_OPENLOOP)
  {
    /* OPENLOOP 状态只累计 ready 保持时间。
     * 条件一旦断掉，保持计数清零，避免偶然一拍满足就接管。
     */
    if (ready_now != 0u)
    {
      if (sw->hold_ticks < hold_ticks)
      {
        sw->hold_ticks++;
      }

      else
      {
        sw->observer_ready = 1u;
        sw->state = FOC_SW_STATE_BLEND;
        sw->blend_ticks = 0u;
        sw->blend_fail_ticks = 0u;
        sw->blend_k = 0.0f;
      }
    }

    else
    {
      sw->observer_ready = 0u;
      sw->hold_ticks = 0u;
      sw->blend_ticks = 0u;
      sw->blend_k = 0.0f;
    }
  }

  else if (sw->state == FOC_SW_STATE_BLEND)
  {
    uint8_t blend_reset_now = 0u;

    /* BLEND 状态做角度平滑过渡。
     * 进入 BLEND 前仍使用严格门限；进入 BLEND 后，对角度单拍毛刺做防抖。
     * 只有连续失败达到阈值，才退回 OPENLOOP。
     */
    if (ready_now == 0u)
    {
      const uint8_t reason = sw->fail_reason;
      const float angle_err_abs = fabsf(FOC_ObserverGetAngleErrDeg(observer));

      if (((reason & (uint8_t)~0x02u) == 0u) &&
          (angle_err_abs <= FOC_SWITCHOVER_BLEND_MAX_ANGLE_ERR_DEG))
      {
        /* BLEND 中只发生角度轻微毛刺，继续过渡，不立刻打回。 */
        sw->blend_fail_ticks = 0u;
      }
      else
      {
        if (sw->blend_fail_ticks < FOC_SWITCHOVER_BLEND_FAIL_HOLD_TICKS)
        {
          sw->blend_fail_ticks++;
        }

        if (sw->blend_fail_ticks >= FOC_SWITCHOVER_BLEND_FAIL_HOLD_TICKS)
        {
          blend_reset_now = 1u;
        }
      }
    }
    else
    {
      sw->blend_fail_ticks = 0u;
    }

    if (blend_reset_now != 0u)
    {
      const uint8_t reset_reason = sw->fail_reason;
      const uint32_t reset_count = sw->blend_reset_count + 1u;
      const float reset_angle_err_deg = FOC_ObserverGetAngleErrDeg(observer);
      FOC_SwitchoverReset(sw);
      sw->last_blend_reset_reason = reset_reason;
      sw->blend_reset_count = reset_count;
      sw->last_blend_reset_angle_err_deg = reset_angle_err_deg;
    }

    else if (blend_ticks == 0u)
    {
      sw->observer_ready = 1u;
      sw->blend_k = 1.0f;
      sw->state = FOC_SW_STATE_OBSERVER;
    }

    else if (sw->blend_ticks < blend_ticks)
    {
      sw->observer_ready = 1u;
      sw->blend_ticks++;
      {
        const float inv_blend_ticks = 1.0f / (float)blend_ticks;
        sw->blend_k = (float)sw->blend_ticks * inv_blend_ticks;
      }
    }

    else
    {
      sw->observer_ready = 1u;
      sw->blend_k = 1.0f;
      sw->state = FOC_SW_STATE_OBSERVER;
    }
  }

  else
  {
    sw->observer_ready = 1u;
    sw->blend_k = 1.0f;
  }

#else

  (void)openloop;
  (void)observer;
  FOC_SwitchoverReset(sw);
#endif
}

float FOC_SwitchoverSelectTheta(const FOC_SwitchoverData_t *sw,
                                const FOC_OpenLoopData_t *openloop,
                                const FOC_ObserverData_t *observer)
{
  float theta_ctrl = FOC_OpenLoopGetThetaErad(openloop);

  if ((sw == 0) || (openloop == 0) || (observer == 0))
  {
    return theta_ctrl;
  }

  if (sw->state == FOC_SW_STATE_BLEND)
  {
    /* 角度混合必须沿最短角差方向走。
     * 所以先 WrapPi 得到 [-pi, pi] 范围内的 delta，再叠加到开环角上。
     */
    const float delta = FOC_MathWrapPi(FOC_ObserverGetThetaRad(observer) - FOC_OpenLoopGetThetaErad(openloop));
    theta_ctrl = FOC_MathWrap2Pi(FOC_OpenLoopGetThetaErad(openloop) + (sw->blend_k * delta));
  }

  else if (sw->state == FOC_SW_STATE_OBSERVER)
  {
    theta_ctrl = FOC_MathWrap2Pi(FOC_ObserverGetThetaRad(observer));
  }

  return theta_ctrl;
}

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
                            float *obs_speed_rad_s)
{
  if (sw == 0)
  {
    return;
  }

  if (sw_state) *sw_state = (uint8_t)sw->state;
  if (observer_ready) *observer_ready = sw->observer_ready;
  if (blend_k) *blend_k = sw->blend_k;
  if (theta_open) *theta_open = FOC_OpenLoopGetThetaErad(openloop);
  if (theta_obs) *theta_obs = FOC_ObserverGetThetaRad(observer);
  if (theta_ctrl_out) *theta_ctrl_out = theta_ctrl;
  if (angle_err_deg) *angle_err_deg = FOC_ObserverGetAngleErrDeg(observer);
  if (open_speed_rad_s) *open_speed_rad_s = FOC_OpenLoopGetElecSpeedRadPerSec(openloop);
  if (obs_speed_rad_s) *obs_speed_rad_s = FOC_ObserverGetSpeedRadPerSec(observer);
}
