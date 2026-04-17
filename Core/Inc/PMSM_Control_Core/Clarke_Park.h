#ifndef FOC_SENSORLESS_CLARKE_PARK_H
#define FOC_SENSORLESS_CLARKE_PARK_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float alpha;
  float beta;
} ClarkePark_AlphaBeta_t;

typedef struct
{
  float d;
  float q;
} ClarkePark_DQ_t;

void ClarkePark_Clarke(float ia, float ib, float ic, ClarkePark_AlphaBeta_t *out);
void ClarkePark_Park(const ClarkePark_AlphaBeta_t *in, float theta_rad, ClarkePark_DQ_t *out);
void ClarkePark_InvPark(const ClarkePark_DQ_t *in, float theta_rad, ClarkePark_AlphaBeta_t *out);
void ClarkePark_InvClarke(const ClarkePark_AlphaBeta_t *in, float *va, float *vb, float *vc);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SENSORLESS_CLARKE_PARK_H */
