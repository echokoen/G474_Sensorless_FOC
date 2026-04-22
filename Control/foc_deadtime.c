#include "foc_deadtime.h"
#include "foc_config.h"

static float foc_deadtime_clampf(float x, float lo, float hi)
{
  if (x > hi) { return hi; }
  if (x < lo) { return lo; }
  return x;
}

static float foc_deadtime_smooth_sign(float i_a, float ith_a)
{
  if (ith_a <= 1.0e-6f)
  {
    if (i_a > 0.0f) { return 1.0f; }
    if (i_a < 0.0f) { return -1.0f; }
    return 0.0f;
  }

  if (i_a >= ith_a) { return 1.0f; }
  if (i_a <= -ith_a) { return -1.0f; }
  {
    const float inv_ith_a = 1.0f / ith_a;
    return i_a * inv_ith_a;
  }
}

void FOC_DeadtimeComp_Init(FOC_DeadtimeComp_t *dt)
{
  FOC_DeadtimeComp_Reset(dt);
}

void FOC_DeadtimeComp_Reset(FOC_DeadtimeComp_t *dt)
{
  if (dt == 0) { return; }
  dt->k_used = 0.0f;
  dt->vdt_u = 0.0f;
  dt->vdt_v = 0.0f;
  dt->vdt_w = 0.0f;
  dt->valpha_out = 0.0f;
  dt->vbeta_out = 0.0f;
}

void FOC_DeadtimeComp_Apply(FOC_DeadtimeComp_t *dt,
                            float vbus_v,
                            float iu_a,
                            float iv_a,
                            float iw_a,
                            float valpha_in,
                            float vbeta_in,
                            float *valpha_out,
                            float *vbeta_out)
{
  const float sqrt3_half = 0.86602540378f;
  const float inv_sqrt3 = 0.57735026919f;
  float va, vb, vc, su, sv, sw, vcm, k_used, vdt_u, vdt_v, vdt_w;

  if ((dt == 0) || (valpha_out == 0) || (vbeta_out == 0))
  {
    return;
  }

  *valpha_out = valpha_in;
  *vbeta_out = vbeta_in;
  FOC_DeadtimeComp_Reset(dt);

  if (vbus_v < FOC_DT_COMP_MIN_VBUS_V)
  {
    dt->valpha_out = *valpha_out;
    dt->vbeta_out = *vbeta_out;
    return;
  }

  k_used = foc_deadtime_clampf(FOC_DT_COMP_K_BASE, 0.0f, FOC_DT_COMP_K_MAX);
  if (k_used <= 0.0f)
  {
    dt->valpha_out = *valpha_out;
    dt->vbeta_out = *vbeta_out;
    return;
  }

  va = valpha_in;
  vb = (-0.5f * valpha_in) + (sqrt3_half * vbeta_in);
  vc = (-0.5f * valpha_in) - (sqrt3_half * vbeta_in);

  su = foc_deadtime_smooth_sign(iu_a, FOC_DT_COMP_CURR_TH_A);
  sv = foc_deadtime_smooth_sign(iv_a, FOC_DT_COMP_CURR_TH_A);
  sw = foc_deadtime_smooth_sign(iw_a, FOC_DT_COMP_CURR_TH_A);

  vdt_u = FOC_DT_COMP_SIGN * k_used * vbus_v * su;
  vdt_v = FOC_DT_COMP_SIGN * k_used * vbus_v * sv;
  vdt_w = FOC_DT_COMP_SIGN * k_used * vbus_v * sw;

  vcm = (vdt_u + vdt_v + vdt_w) * 0.33333333333f;
  vdt_u -= vcm;
  vdt_v -= vcm;
  vdt_w -= vcm;

  va += vdt_u;
  vb += vdt_v;
  vc += vdt_w;

  *valpha_out = va;
  *vbeta_out = (vb - vc) * inv_sqrt3;

  dt->k_used = k_used;
  dt->vdt_u = vdt_u;
  dt->vdt_v = vdt_v;
  dt->vdt_w = vdt_w;
  dt->valpha_out = *valpha_out;
  dt->vbeta_out = *vbeta_out;
}
