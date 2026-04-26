#include "foc_speed_loop.h"
#include "foc_config.h"
#include "stm32g4xx_hal.h"
#include <math.h>

static float foc_speed_loop_clampf(float x, float lo, float hi)
{
  if (x > hi)
  {
    return hi;
  }
  if (x < lo)
  {
    return lo;
  }
  return x;
}

static float foc_speed_loop_elec_rad_s_to_mech_rad_s(float elec_rad_s)
{
  const float inv_pole_pairs = 1.0f / FOC_POLE_PAIRS;
  return elec_rad_s * inv_pole_pairs;
}

static float foc_speed_loop_ramp_to_target(float current, float target, float step_abs)
{
  if (step_abs <= 0.0f)
  {
    return target;
  }

  if (current < target)
  {
    current += step_abs;
    if (current > target)
    {
      current = target;
    }
  }
  else if (current > target)
  {
    current -= step_abs;
    if (current < target)
    {
      current = target;
    }
  }

  return current;
}

void FOC_SpeedLoopInit(FOC_SpeedLoopData_t *speed_loop)
{
  if (speed_loop == 0)
  {
    return;
  }

  PIController_Init(&speed_loop->pi,
                    FOC_SPEED_KP,
                    FOC_SPEED_KI,
                    FOC_SPEED_IQ_MIN_A,
                    FOC_SPEED_IQ_MAX_A);
  PIController_Reset(&speed_loop->pi);
  speed_loop->speed_ref_cmd_mech_rad_s = 0.0f;
  speed_loop->speed_target_mech_rad_s = 0.0f;
  speed_loop->speed_fdb_mech_rad_s = 0.0f;
  speed_loop->speed_fdb_raw_mech_rad_s = 0.0f;
  speed_loop->iq_ref_cmd_a = 0.0f;
  speed_loop->run_entry_tick_ms = 0u;
  speed_loop->speed_enable_tick_ms = 0u;
  speed_loop->enabled = 0u;
}

void FOC_SpeedLoopReset(FOC_SpeedLoopData_t *speed_loop, float speed_init_mech_rad_s)
{
  if (speed_loop == 0)
  {
    return;
  }

  PIController_Reset(&speed_loop->pi);
  speed_loop->speed_fdb_mech_rad_s = speed_init_mech_rad_s;
  speed_loop->speed_fdb_raw_mech_rad_s = speed_init_mech_rad_s;
  speed_loop->speed_ref_cmd_mech_rad_s = speed_init_mech_rad_s;
  speed_loop->speed_target_mech_rad_s = speed_init_mech_rad_s;
  speed_loop->iq_ref_cmd_a = FOC_IQ_REF_A;
  speed_loop->run_entry_tick_ms = HAL_GetTick();
  speed_loop->speed_enable_tick_ms = 0u;
  speed_loop->enabled = 0u;
}

void FOC_SpeedLoopPrepareRunEntry(FOC_SpeedLoopData_t *speed_loop,
                                  float observer_elec_rad_s,
                                  float fallback_open_elec_rad_s)
{
  float speed_init_mech_rad_s = foc_speed_loop_elec_rad_s_to_mech_rad_s(observer_elec_rad_s);

  if (fabsf(speed_init_mech_rad_s) < 1.0f)
  {
    speed_init_mech_rad_s = foc_speed_loop_elec_rad_s_to_mech_rad_s(fallback_open_elec_rad_s);
  }

  FOC_SpeedLoopReset(speed_loop, speed_init_mech_rad_s);
}

void FOC_SpeedLoopSetTargetMechRadS(FOC_SpeedLoopData_t *speed_loop, float target_mech_rad_s)
{
  if (speed_loop == 0)
  {
    return;
  }

  speed_loop->speed_target_mech_rad_s = target_mech_rad_s;
}

void FOC_SpeedLoopStep(FOC_SpeedLoopData_t *speed_loop,
                       float observer_elec_rad_s,
                       float dt_sec)
{
  if (speed_loop == 0)
  {
    return;
  }

#if (FOC_SPEED_LOOP_ENABLE_IN_RUN == 0u)
  speed_loop->speed_fdb_raw_mech_rad_s = foc_speed_loop_elec_rad_s_to_mech_rad_s(observer_elec_rad_s);
  speed_loop->speed_fdb_mech_rad_s = speed_loop->speed_fdb_raw_mech_rad_s;
  speed_loop->iq_ref_cmd_a = FOC_IQ_REF_A;
  speed_loop->enabled = 0u;
  return;
#else
  {
    const float speed_alpha = foc_speed_loop_clampf(FOC_SPEED_FDB_FILTER_ALPHA, 0.0f, 1.0f);
    const uint32_t now_ms = HAL_GetTick();
    float iq_cmd;

    dt_sec = foc_speed_loop_clampf(dt_sec, 0.001f, 0.100f);

    speed_loop->speed_fdb_raw_mech_rad_s =
        foc_speed_loop_elec_rad_s_to_mech_rad_s(observer_elec_rad_s);
    speed_loop->speed_fdb_mech_rad_s += speed_alpha *
        (speed_loop->speed_fdb_raw_mech_rad_s - speed_loop->speed_fdb_mech_rad_s);

    if (speed_loop->enabled == 0u)
    {
      /* Hold I/F torque briefly after observer takeover. */
      speed_loop->iq_ref_cmd_a = FOC_IQ_REF_A;

      if ((now_ms - speed_loop->run_entry_tick_ms) >= FOC_SPEED_ENABLE_DELAY_MS)
      {
        PIController_Reset(&speed_loop->pi);
        speed_loop->speed_enable_tick_ms = now_ms;
        speed_loop->enabled = 1u;
      }
      return;
    }

    speed_loop->speed_ref_cmd_mech_rad_s = foc_speed_loop_ramp_to_target(
        speed_loop->speed_ref_cmd_mech_rad_s,
        speed_loop->speed_target_mech_rad_s,
        FOC_SPEED_REF_RAMP_HZ_PER_S * 6.28318530718f * dt_sec);

    iq_cmd = PIController_Run(&speed_loop->pi,
                              speed_loop->speed_ref_cmd_mech_rad_s,
                              speed_loop->speed_fdb_mech_rad_s,
                              dt_sec);

    if ((now_ms - speed_loop->speed_enable_tick_ms) < FOC_SPEED_TAKEOVER_HOLD_MS)
    {
      if (iq_cmd < FOC_SPEED_TAKEOVER_IQ_HOLD_A)
      {
        iq_cmd = FOC_SPEED_TAKEOVER_IQ_HOLD_A;
      }
    }

    {
      const float iq_target_a = foc_speed_loop_clampf(iq_cmd,
                                                      FOC_SPEED_IQ_MIN_A,
                                                      FOC_SPEED_IQ_MAX_A);
      const float iq_step_a = FOC_SPEED_IQ_SLEW_A_PER_S * dt_sec;

      if (iq_target_a > (speed_loop->iq_ref_cmd_a + iq_step_a))
      {
        speed_loop->iq_ref_cmd_a += iq_step_a;
      }
      else if (iq_target_a < (speed_loop->iq_ref_cmd_a - iq_step_a))
      {
        speed_loop->iq_ref_cmd_a -= iq_step_a;
      }
      else
      {
        speed_loop->iq_ref_cmd_a = iq_target_a;
      }

      speed_loop->iq_ref_cmd_a = foc_speed_loop_clampf(speed_loop->iq_ref_cmd_a,
                                                       FOC_SPEED_IQ_MIN_A,
                                                       FOC_SPEED_IQ_MAX_A);
    }
  }
#endif
}

uint8_t FOC_SpeedLoopIsEnabled(const FOC_SpeedLoopData_t *speed_loop)
{
  if (speed_loop == 0)
  {
    return 0u;
  }

  return speed_loop->enabled;
}

void FOC_SpeedLoopGetSnapshot(const FOC_SpeedLoopData_t *speed_loop,
                              FOC_SpeedLoopSnapshot_t *snapshot)
{
  if ((speed_loop == 0) || (snapshot == 0))
  {
    return;
  }

  snapshot->speed_ref_mech_rad_s = speed_loop->speed_ref_cmd_mech_rad_s;
  snapshot->speed_target_mech_rad_s = speed_loop->speed_target_mech_rad_s;
  snapshot->speed_fdb_mech_rad_s = speed_loop->speed_fdb_mech_rad_s;
  snapshot->iq_ref_cmd_a = speed_loop->iq_ref_cmd_a;
  snapshot->enabled = FOC_SpeedLoopIsEnabled(speed_loop);
}
