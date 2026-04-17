#include "PMSM_Control_Core/PI_Controller.h"

static float PIController_Clamp(float value, float min_value, float max_value)
{
  if (value > max_value) return max_value;
  if (value < min_value) return min_value;
  return value;
}

void PIController_Init(PIController_t *pi, float kp, float ki, float out_min, float out_max)
{
  if (pi == 0)
  {
    return;
  }

  pi->kp = kp;
  pi->ki = ki;
  pi->integral = 0.0f;
  pi->integral_min = out_min;
  pi->integral_max = out_max;
  pi->out_min = out_min;
  pi->out_max = out_max;
}

void PIController_Reset(PIController_t *pi)
{
  if (pi == 0)
  {
    return;
  }

  pi->integral = 0.0f;
}

float PIController_Run(PIController_t *pi, float ref, float meas, float dt_sec)
{
  if (pi == 0)
  {
    return 0.0f;
  }

  const float err = ref - meas;
  const float p_term = pi->kp * err;
  float integral_next = pi->integral + (pi->ki * err * dt_sec);
  integral_next = PIController_Clamp(integral_next, pi->integral_min, pi->integral_max);

  const float unsat = p_term + integral_next;
  const float out = PIController_Clamp(unsat, pi->out_min, pi->out_max);

  if ((unsat == out) ||
      ((unsat > pi->out_max) && (err < 0.0f)) ||
      ((unsat < pi->out_min) && (err > 0.0f)))
  {
    pi->integral = integral_next;
  }

  return out;
}
