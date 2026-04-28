#ifndef FOC_SENSORLESS_PI_CONTROLLER_H
#define FOC_SENSORLESS_PI_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* 通用离散 PI 控制器。
 *
 * 设计目标：
 * - 电流环 d/q 轴可以复用；
 * - 速度环也可以复用；
 * - 所有 PI 的限幅和抗积分饱和逻辑保持一致。
 *
 * 当前 PI 行为不是 Simulink 默认 PID 的黑盒行为，而是手写离散实现：
 * 1. err = ref - meas；
 * 2. p_term = kp * err；
 * 3. integral_next = integral + ki * err * dt；
 * 4. integral_next 先限幅；
 * 5. output = p_term + integral_next；
 * 6. output 再限幅；
 * 7. 只有未饱和或误差正在把输出拉回时，才更新积分。
 */

typedef struct
{
  /* 比例增益。电流环中通常与电感和目标带宽相关。 */
  float kp;
  /* 积分增益。电流环中通常与电阻和目标带宽相关。 */
  float ki;
  /* 积分状态，跨控制周期保留。 */
  float integral;
  /* 积分项下限，防止积分状态无限累积。 */
  float integral_min;
  /* 积分项上限，防止积分状态无限累积。 */
  float integral_max;
  /* PI 输出下限。 */
  float out_min;
  /* PI 输出上限。 */
  float out_max;
} PIController_t;

/* 初始化 PI 参数和限幅，同时清空积分状态。 */
void PIController_Init(PIController_t *pi, float kp, float ki, float out_min, float out_max);

/* 只清空积分状态，不改变 kp/ki/限幅。状态切换时常用。 */
void PIController_Reset(PIController_t *pi);

/* 执行一拍 PI。
 * dt_sec 必须与调用周期一致，否则积分强度会错。
 */
float PIController_Run(PIController_t *pi, float ref, float meas, float dt_sec);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SENSORLESS_PI_CONTROLLER_H */
