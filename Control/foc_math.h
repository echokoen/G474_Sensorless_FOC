#ifndef __FOC_MATH_H__
#define __FOC_MATH_H__
#ifdef __cplusplus
extern "C" {
#endif

/* 轻量数学工具。
 *
 * 这里只保留非常小的内联函数，供 FOC 各模块共享。
 */

static inline float FOC_MathClampF(float x, float min_v, float max_v)
{
  /* 通用浮点限幅，常用于保护 PI 输出、角度混合系数等。 */
  if (x < min_v) return min_v;
  if (x > max_v) return max_v;
  return x;
}

static inline float FOC_MathWrap2Pi(float angle)
{
  /* 把任意角度折回 [0, 2pi)，适合存储绝对电角度。 */
  const float two_pi = 6.2831853071795864769f;
  while (angle >= two_pi) angle -= two_pi;
  while (angle < 0.0f) angle += two_pi;
  return angle;
}

static inline float FOC_MathWrapPi(float angle)
{
  /* 把角差折回 [-pi, pi]，适合做“最短角度误差”判断。 */
  const float pi = 3.14159265358979323846f;
  const float two_pi = 6.2831853071795864769f;
  while (angle > pi) angle -= two_pi;
  while (angle < -pi) angle += two_pi;
  return angle;
}

static inline float FOC_MathRadToDeg(float rad)
{
  /* 串口调试时角度用 deg 更直观，内部控制仍使用 rad。 */
  return rad * 57.29577951308232f;
}

#ifdef __cplusplus
}

#endif
#endif
