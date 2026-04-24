#include "PMSM_Control_Core/PI_Controller.h"

static float PIController_Clamp(float value, float min_value, float max_value)
{
  /* 小工具函数：把 value 限制在 [min_value, max_value] 范围内。 */
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

  /* 参数只在初始化时写入；运行过程中 PIController_Run 只更新 integral。 */
  pi->kp = kp;
  pi->ki = ki;
  pi->integral = 0.0f;
  /* 当前设计把积分项限幅与输出限幅设为同一范围，简单且安全。 */
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

  /* 误差定义固定为 ref - meas。
   * 如果闭环表现为正反馈，优先检查测量符号或输出极性，而不是改这里。
   */
  const float err = ref - meas;
  const float p_term = pi->kp * err;

  /* 先预测下一拍积分值，并对积分项本身限幅。 */
  float integral_next = pi->integral + (pi->ki * err * dt_sec);
  integral_next = PIController_Clamp(integral_next, pi->integral_min, pi->integral_max);

  /* 未限幅输出 = 比例项 + 积分预测值。 */
  const float unsat = p_term + integral_next;

  /* 最终输出再做一次限幅，避免给后级过大的命令。 */
  const float out = PIController_Clamp(unsat, pi->out_min, pi->out_max);

  /* 抗积分饱和：
   * - 没饱和：允许积分正常更新；
   * - 上饱和且误差为负：误差正在把输出往回拉，允许积分更新；
   * - 下饱和且误差为正：误差正在把输出往回拉，允许积分更新；
   * - 其他饱和情况：冻结积分，避免越积越深。
   */
  if ((unsat == out) ||
      ((unsat > pi->out_max) && (err < 0.0f)) ||
      ((unsat < pi->out_min) && (err > 0.0f)))
  {
    pi->integral = integral_next;
  }

  return out;
}
