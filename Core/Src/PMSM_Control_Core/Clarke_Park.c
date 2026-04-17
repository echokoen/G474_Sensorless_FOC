#include "PMSM_Control_Core/Clarke_Park.h"
#include <math.h>

void ClarkePark_Clarke(float ia, float ib, float ic, ClarkePark_AlphaBeta_t *out)
{
  (void)ic;
  if (out == 0)
  {
    return;
  }

  out->alpha = ia;
  out->beta = (ia + (2.0f * ib)) * 0.5773502691896258f;
}

void ClarkePark_Park(const ClarkePark_AlphaBeta_t *in, float theta_rad, ClarkePark_DQ_t *out)
{
  if ((in == 0) || (out == 0))
  {
    return;
  }

  const float sin_t = sinf(theta_rad);
  const float cos_t = cosf(theta_rad);
  out->d = (cos_t * in->alpha) + (sin_t * in->beta);
  out->q = (-sin_t * in->alpha) + (cos_t * in->beta);
}

void ClarkePark_InvPark(const ClarkePark_DQ_t *in, float theta_rad, ClarkePark_AlphaBeta_t *out)
{
  if ((in == 0) || (out == 0))
  {
    return;
  }

  const float sin_t = sinf(theta_rad);
  const float cos_t = cosf(theta_rad);
  out->alpha = (cos_t * in->d) - (sin_t * in->q);
  out->beta = (sin_t * in->d) + (cos_t * in->q);
}

void ClarkePark_InvClarke(const ClarkePark_AlphaBeta_t *in, float *va, float *vb, float *vc)
{
  if (in == 0)
  {
    return;
  }

  const float sqrt3 = 1.7320508075688772f;
  if (va != 0) *va = in->alpha;
  if (vb != 0) *vb = (-0.5f * in->alpha) + (0.5f * sqrt3 * in->beta);
  if (vc != 0) *vc = (-0.5f * in->alpha) - (0.5f * sqrt3 * in->beta);
}
