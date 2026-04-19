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
  if (x < min_v) return min_v;
  if (x > max_v) return max_v;
  return x;
}

static inline float FOC_MathWrap2Pi(float angle)
{
  const float two_pi = 6.2831853071795864769f;
  while (angle >= two_pi) angle -= two_pi;
  while (angle < 0.0f) angle += two_pi;
  return angle;
}

static inline float FOC_MathWrapPi(float angle)
{
  const float pi = 3.14159265358979323846f;
  const float two_pi = 6.2831853071795864769f;
  while (angle > pi) angle -= two_pi;
  while (angle < -pi) angle += two_pi;
  return angle;
}

static inline float FOC_MathRadToDeg(float rad)
{
  return rad * 57.29577951308232f;
}

#ifdef __cplusplus
}

#endif
#endif
