#include "foc_handover.h"
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
static uint8_t foc_handover_condition(FOC_Handover_t *handover,
                                        const FOC_Startup_t *startup,
                                        const FOC_ObserverData_t *observer)
{
  const float open_speed = FOC_Startup_GetElecSpeedRadPerSec(startup);
  const float obs_speed = FOC_ObserverGetSpeedRadPerSec(observer);
  const float speed_err = fabsf(obs_speed - open_speed);
  const float angle_err_rad = FOC_MathWrapPi(FOC_Startup_GetThetaErad(startup) -
                                             FOC_ObserverGetThetaRad(observer));
  const float angle_err_deg = FOC_MathRadToDeg(angle_err_rad);
  float speed_limit = fabsf(open_speed) * FOC_HANDOVER_MAX_SPEED_ERR_RATIO;

  if (handover != 0)
  {
    handover->angle_err_deg = angle_err_deg;
    handover->open_speed_rad_s = open_speed;
    handover->obs_speed_rad_s = obs_speed;
    handover->speed_err_rad_s = speed_err;
    handover->ready_now = 0u;
    handover->fail_reason = 0u;
  }

  /* 低速时开环电角速度本身较小，
   * 如果只按比例门限判断，速度误差窗口会过于苛刻，
   * 容易导致明明 observer 已经基本跟上，却迟迟不给接管。 */
  if (speed_limit < FOC_HANDOVER_MAX_SPEED_ERR_MIN_RAD_S)
  {
    speed_limit = FOC_HANDOVER_MAX_SPEED_ERR_MIN_RAD_S;
  }

  if (FOC_Startup_GetFreqHz(startup) < FOC_HANDOVER_MIN_MECH_FREQ_HZ)
  {
    if (handover != 0) { handover->fail_reason |= 0x01u; }
    return 0u;
  }

  if (fabsf(angle_err_deg) > FOC_HANDOVER_MAX_ANGLE_ERR_DEG)
  {
    if (handover != 0) { handover->fail_reason |= 0x02u; }
    return 0u;
  }

  /* 速度判定：
   * 只有当 observer 速度与开环速度足够接近时，
   * 才允许进入 handover ready。
   * 这样可以避免“角度偶然对上，但速度其实没跟住”导致的误接管。 */
  if (speed_err > speed_limit)
  {
    if (handover != 0) { handover->fail_reason |= 0x04u; }
    return 0u;
  }

  if (handover != 0)
  {
    handover->ready_now = 1u;
  }

  return 1u;
}

static void foc_handover_update_observer_lock(FOC_Handover_t *handover,
                                                const FOC_Startup_t *startup,
                                                const FOC_ObserverData_t *observer)
{
  const float angle_err_rad = FOC_MathWrapPi(FOC_Startup_GetThetaErad(startup) -
                                             FOC_ObserverGetThetaRad(observer));
  const float angle_err_deg = FOC_MathRadToDeg(angle_err_rad);
  const uint32_t lock_hold_ticks =
      (uint32_t)(FOC_OBS_LOCK_HOLD_MS / (FOC_TS_SEC * 1000.0f));

  if (handover == 0)
  {
    return;
  }

  handover->angle_err_deg = angle_err_deg;

  /* Migrated from FOC_ObserverUpdate(): this is the legacy observer locked
   * condition, kept for debug/compatibility but not used as the final takeover
   * state. The handover state machine still owns observer_ready.
   */
  if ((FOC_Startup_GetFreqHz(startup) >= FOC_OBS_ENABLE_FREQ_HZ) &&
      (fabsf(handover->angle_err_deg) < FOC_OBS_LOCK_ERR_DEG))
  {
    if (handover->lock_hold_ticks < lock_hold_ticks)
    {
      handover->lock_hold_ticks++;
    }
    else
    {
      handover->locked = 1u;
    }
  }
  else
  {
    handover->lock_hold_ticks = 0u;
    handover->locked = 0u;
  }
}

void FOC_Handover_Init(FOC_Handover_t *handover)
{
  FOC_Handover_Reset(handover);
}

void FOC_Handover_Reset(FOC_Handover_t *handover)
{
  if (handover == 0)
  {
    return;
  }

  handover->observer_ready = 0u;
  handover->ready_now = 0u;
  handover->locked = 0u;
  handover->hold_ticks = 0u;
  handover->lock_hold_ticks = 0u;
  handover->blend_ticks = 0u;
  handover->blend_k = 0.0f;
  handover->angle_err_deg = 0.0f;
  handover->open_speed_rad_s = 0.0f;
  handover->obs_speed_rad_s = 0.0f;
  handover->speed_err_rad_s = 0.0f;
  handover->fail_reason = 0u;
  handover->last_blend_reset_reason = 0u;
  handover->blend_fail_ticks = 0u;
  handover->blend_reset_count = 0u;
  handover->last_blend_reset_angle_err_deg = 0.0f;
  handover->state = FOC_HANDOVER_STATE_OPENLOOP;
}

void FOC_Handover_Update(FOC_Handover_t *handover,
                          const FOC_Startup_t *startup,
                          const FOC_ObserverData_t *observer)
{
#if (FOC_HANDOVER_ENABLE != 0u)

  const float inv_ctrl_period_ms = 1.0f / (FOC_TS_SEC * 1000.0f);
  const uint32_t hold_ticks = (uint32_t)(FOC_HANDOVER_HOLD_MS * inv_ctrl_period_ms);
  const uint32_t blend_ticks = (uint32_t)(FOC_HANDOVER_BLEND_MS * inv_ctrl_period_ms);
  uint8_t ready_now;

  if ((handover == 0) || (startup == 0) || (observer == 0))
  {
    return;
  }

  foc_handover_update_observer_lock(handover, startup, observer);
  ready_now = foc_handover_condition(handover, startup, observer);

  if (handover->state == FOC_HANDOVER_STATE_OPENLOOP)
  {
    /* OPENLOOP 状态只累计 ready 保持时间。
     * 条件一旦断掉，保持计数清零，避免偶然一拍满足就接管。
     */
    if (ready_now != 0u)
    {
      if (handover->hold_ticks < hold_ticks)
      {
        handover->hold_ticks++;
      }

      else
      {
        handover->observer_ready = 1u;
        handover->state = FOC_HANDOVER_STATE_BLEND;
        handover->blend_ticks = 0u;
        handover->blend_fail_ticks = 0u;
        handover->blend_k = 0.0f;
      }
    }

    else
    {
      handover->observer_ready = 0u;
      handover->hold_ticks = 0u;
      handover->blend_ticks = 0u;
      handover->blend_k = 0.0f;
    }
  }

  else if (handover->state == FOC_HANDOVER_STATE_BLEND)
  {
    uint8_t blend_reset_now = 0u;

    /* BLEND 状态做角度平滑过渡。
     * 进入 BLEND 前仍使用严格门限；进入 BLEND 后，对角度单拍毛刺做防抖。
     * 只有连续失败达到阈值，才退回 OPENLOOP。
     */
    if (ready_now == 0u)
    {
      const uint8_t reason = handover->fail_reason;
      const float angle_err_abs = fabsf(handover->angle_err_deg);

      if (((reason & (uint8_t)~0x02u) == 0u) &&
          (angle_err_abs <= FOC_HANDOVER_BLEND_MAX_ANGLE_ERR_DEG))
      {
        /* BLEND 中只发生角度轻微毛刺，继续过渡，不立刻打回。 */
        handover->blend_fail_ticks = 0u;
      }
      else
      {
        if (handover->blend_fail_ticks < FOC_HANDOVER_BLEND_FAIL_HOLD_TICKS)
        {
          handover->blend_fail_ticks++;
        }

        if (handover->blend_fail_ticks >= FOC_HANDOVER_BLEND_FAIL_HOLD_TICKS)
        {
          blend_reset_now = 1u;
        }
      }
    }
    else
    {
      handover->blend_fail_ticks = 0u;
    }

    if (blend_reset_now != 0u)
    {
      const uint8_t reset_reason = handover->fail_reason;
      const uint32_t reset_count = handover->blend_reset_count + 1u;
      const float reset_angle_err_deg = handover->angle_err_deg;
      FOC_Handover_Reset(handover);
      handover->last_blend_reset_reason = reset_reason;
      handover->blend_reset_count = reset_count;
      handover->last_blend_reset_angle_err_deg = reset_angle_err_deg;
    }

    else if (blend_ticks == 0u)
    {
      handover->observer_ready = 1u;
      handover->blend_k = 1.0f;
      handover->state = FOC_HANDOVER_STATE_OBSERVER;
    }

    else if (handover->blend_ticks < blend_ticks)
    {
      handover->observer_ready = 1u;
      handover->blend_ticks++;
      {
        const float inv_blend_ticks = 1.0f / (float)blend_ticks;
        handover->blend_k = (float)handover->blend_ticks * inv_blend_ticks;
      }
    }

    else
    {
      handover->observer_ready = 1u;
      handover->blend_k = 1.0f;
      handover->state = FOC_HANDOVER_STATE_OBSERVER;
    }
  }

  else
  {
    handover->observer_ready = 1u;
    handover->blend_k = 1.0f;
  }

#else

  (void)startup;
  (void)observer;
  FOC_Handover_Reset(handover);
#endif
}

float FOC_Handover_SelectTheta(const FOC_Handover_t *handover,
                                const FOC_Startup_t *startup,
                                const FOC_ObserverData_t *observer)
{
  float theta_ctrl = FOC_Startup_GetThetaErad(startup);

  if ((handover == 0) || (startup == 0) || (observer == 0))
  {
    return theta_ctrl;
  }

  if (handover->state == FOC_HANDOVER_STATE_BLEND)
  {
    /* 角度混合必须沿最短角差方向走。
     * 所以先 WrapPi 得到 [-pi, pi] 范围内的 delta，再叠加到开环角上。
     */
    const float delta = FOC_MathWrapPi(FOC_ObserverGetThetaRad(observer) - FOC_Startup_GetThetaErad(startup));
    theta_ctrl = FOC_MathWrap2Pi(FOC_Startup_GetThetaErad(startup) + (handover->blend_k * delta));
  }

  else if (handover->state == FOC_HANDOVER_STATE_OBSERVER)
  {
    theta_ctrl = FOC_MathWrap2Pi(FOC_ObserverGetThetaRad(observer));
  }

  return theta_ctrl;
}

void FOC_Handover_GetDebug(const FOC_Handover_t *handover,
                            const FOC_Startup_t *startup,
                            const FOC_ObserverData_t *observer,
                            float theta_ctrl,
                            uint8_t *handover_state,
                            uint8_t *observer_ready,
                            float *blend_k,
                            float *theta_open,
                            float *theta_obs,
                            float *theta_ctrl_out,
                            float *angle_err_deg,
                            float *open_speed_rad_s,
                            float *obs_speed_rad_s)
{
  if (handover == 0)
  {
    return;
  }

  if (handover_state) *handover_state = (uint8_t)handover->state;
  if (observer_ready) *observer_ready = handover->observer_ready;
  if (blend_k) *blend_k = handover->blend_k;
  if (theta_open) *theta_open = FOC_Startup_GetThetaErad(startup);
  if (theta_obs) *theta_obs = FOC_ObserverGetThetaRad(observer);
  if (theta_ctrl_out) *theta_ctrl_out = theta_ctrl;
  if (angle_err_deg) *angle_err_deg = handover->angle_err_deg;
  if (open_speed_rad_s) *open_speed_rad_s = FOC_Startup_GetElecSpeedRadPerSec(startup);
  if (obs_speed_rad_s) *obs_speed_rad_s = FOC_ObserverGetSpeedRadPerSec(observer);
}
