#include "PMSM_Control_Core/Clarke_Park.h"
#include <math.h>

/* 这一版直接按 F407 工程里的 Clarke/Park/InvPark 公式重写。
 *
 * 目标不是改接口，而是把当前 G474 工程底层数学实现替换成
 * 与 F407 工程一致的版本，减少后续联调时的算法差异。
 */
void ClarkePark_Clarke(float ia, float ib, float ic, ClarkePark_AlphaBeta_t *out)
{
  (void)ic;
  if (out == 0)
  {
    return;
  }

  /* F407 版本：
   * alpha = a
   * beta  = (a + 2b) / sqrt(3)
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

  /* F407 版本：
   * d =  alpha * cos(theta) + beta * sin(theta)
   * q = -alpha * sin(theta) + beta * cos(theta)
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

  /* F407 版本：
   * alpha = d * cos(theta) - q * sin(theta)
   * beta  = d * sin(theta) + q * cos(theta)
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

  if (va != 0) *va = in->alpha;
  if (vb != 0) *vb = (-0.5f * in->alpha) + (0.8660254f * in->beta);
  if (vc != 0) *vc = (-0.5f * in->alpha) - (0.8660254f * in->beta);
}
