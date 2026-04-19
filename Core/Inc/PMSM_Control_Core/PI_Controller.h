#ifndef FOC_SENSORLESS_PI_CONTROLLER_H
#define FOC_SENSORLESS_PI_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* 通用 PI 控制器。
 *
 * 设计目标是让电流环、速度环或后续其他控制环都能复用同一套 PI 实现。
 */

typedef struct
{
  float kp;
  float ki;
  float integral;
  float integral_min;
  float integral_max;
  float out_min;
  float out_max;
} PIController_t;

void PIController_Init(PIController_t *pi, float kp, float ki, float out_min, float out_max);
void PIController_Reset(PIController_t *pi);
float PIController_Run(PIController_t *pi, float ref, float meas, float dt_sec);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SENSORLESS_PI_CONTROLLER_H */
