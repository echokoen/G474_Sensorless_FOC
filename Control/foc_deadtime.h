#ifndef __FOC_DEADTIME_H__
#define __FOC_DEADTIME_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "foc_types.h"

typedef struct
{
  float k_used;
  float vdt_u;
  float vdt_v;
  float vdt_w;
  float valpha_out;
  float vbeta_out;
} FOC_DeadtimeComp_t;

void FOC_DeadtimeComp_Init(FOC_DeadtimeComp_t *dt);
void FOC_DeadtimeComp_Reset(FOC_DeadtimeComp_t *dt);

void FOC_DeadtimeComp_Apply(FOC_DeadtimeComp_t *dt,
                            float vbus_v,
                            float iu_a,
                            float iv_a,
                            float iw_a,
                            float valpha_in,
                            float vbeta_in,
                            float *valpha_out,
                            float *vbeta_out);

#ifdef __cplusplus
}
#endif
#endif
