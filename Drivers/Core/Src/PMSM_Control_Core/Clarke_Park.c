#include "PMSM_Control_Core/Clarke_Park.h"
#include <math.h>

/* 坐标变换实现。
 *
 * 注意事项：
 * - 这里所有函数都只做纯数学变换，不保存状态；
 * - 输入为空时直接返回，避免中断路径里出现空指针 HardFault；
 * - Park 和 InvPark 必须成对使用同一套符号约定，否则 d/q 会互相串轴。
 */
void ClarkePark_Clarke(float ia, float ib, float ic, ClarkePark_AlphaBeta_t *out)
{
  /* 当前 Clarke 公式只需要 ia/ib，ic 参数用于保持三相接口完整。 */
  (void)ic;
  if (out == 0)
  {
    return;
  }

  /* Clarke 变换：
   * alpha = a
   * beta  = (a + 2b) / sqrt(3)
   *
   * 该形式隐含 ia + ib + ic = 0，因此只用两相即可恢复静止坐标。
   */
  out->alpha = ia;
  out->beta = (ia + (2.0f * ib)) * 0.577350269f;
}

void ClarkePark_Park(const ClarkePark_AlphaBeta_t *in, float theta_rad, ClarkePark_DQ_t *out)
{
  float sin_t;
  float cos_t;

  if ((in == 0) || (out == 0))
  {
    return;
  }

  sin_t = sinf(theta_rad);
  cos_t = cosf(theta_rad);

  /* Park 变换：
   * d =  alpha * cos(theta) + beta * sin(theta)
   * q = -alpha * sin(theta) + beta * cos(theta)
   *
   * theta_rad 增加时，dq 坐标系相对 alpha/beta 正向旋转。
   * 如果实际电机表现为 q 轴方向反了，优先检查角度方向和相序。
   */
  out->d = (in->alpha * cos_t) + (in->beta * sin_t);
  out->q = (-in->alpha * sin_t) + (in->beta * cos_t);
}

void ClarkePark_InvPark(const ClarkePark_DQ_t *in, float theta_rad, ClarkePark_AlphaBeta_t *out)
{
  float sin_t;
  float cos_t;

  if ((in == 0) || (out == 0))
  {
    return;
  }

  sin_t = sinf(theta_rad);
  cos_t = cosf(theta_rad);

  /* 反 Park 变换：
   * alpha = d * cos(theta) - q * sin(theta)
   * beta  = d * sin(theta) + q * cos(theta)
   *
   * 这是 Park 的逆运算，电流环的 vd/vq 就通过这里变成静止坐标电压。
   */
  out->alpha = (in->d * cos_t) - (in->q * sin_t);
  out->beta = (in->d * sin_t) + (in->q * cos_t);
}

void ClarkePark_InvClarke(const ClarkePark_AlphaBeta_t *in, float *va, float *vb, float *vc)
{
  if (in == 0)
  {
    return;
  }

  /* 反 Clarke：
   * 把 alpha/beta 还原成三相对称量，三相和理论上为 0。
   * 当前 SVPWM 主路径直接吃 alpha/beta，所以这里更多用于调试和模型对照。
   */
  if (va != 0) *va = in->alpha;
  if (vb != 0) *vb = (-0.5f * in->alpha) + (0.8660254f * in->beta);
  if (vc != 0) *vc = (-0.5f * in->alpha) - (0.8660254f * in->beta);
}
