#ifndef FOC_SENSORLESS_CLARKE_PARK_H
#define FOC_SENSORLESS_CLARKE_PARK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Clarke / Park / 反变换基础数学接口。
 *
 * 第一轮直接复用这一层作为统一坐标变换底座，
 * 让上层的 sampling / current_loop / observer 不再各自重复实现。
 */

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
