#include "foc_switchover.h"
#include "foc_config.h"
#include "foc_math.h"
#include <math.h>

/* 观测器接管与过渡实现。
 *
 * 负责判断接管条件、执行混合过渡并输出最终控制角。
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
    return 0u;
  }

  if (fabsf(FOC_ObserverGetAngleErrDeg(observer)) > FOC_SWITCHOVER_MAX_ANGLE_ERR_DEG)
  {
    return 0u;
  }

  /* 速度判定：
   * 只有当 observer 速度与开环速度足够接近时，
   * 才允许进入 switchover ready。
   * 这样可以避免“角度偶然对上，但速度其实没跟住”导致的误接管。 */
  if (speed_err > speed_limit)
  {
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
  sw->state = FOC_SW_STATE_OPENLOOP;
}

void FOC_SwitchoverUpdate(FOC_SwitchoverData_t *sw,
                          const FOC_OpenLoopData_t *openloop,
                          const FOC_ObserverData_t *observer)
{
#if (FOC_SWITCHOVER_ENABLE != 0u)

  const uint32_t hold_ticks = (uint32_t)(FOC_SWITCHOVER_HOLD_MS / (FOC_TS_SEC * 1000.0f));
  const uint32_t blend_ticks = (uint32_t)(FOC_SWITCHOVER_BLEND_MS / (FOC_TS_SEC * 1000.0f));
  const uint8_t ready_now = foc_switchover_condition(sw, openloop, observer);

  if ((sw == 0) || (openloop == 0) || (observer == 0))
  {
    return;
  }

  if (sw->state == FOC_SW_STATE_OPENLOOP)
  {
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
    if (ready_now == 0u)
    {
      FOC_SwitchoverReset(sw);
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
      sw->blend_k = (float)sw->blend_ticks / (float)blend_ticks;
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
