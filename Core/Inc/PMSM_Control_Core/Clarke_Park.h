#ifndef FOC_SENSORLESS_CLARKE_PARK_H
#define FOC_SENSORLESS_CLARKE_PARK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Clarke / Park / 反变换基础数学接口。
 *
 * 这个文件只放“坐标变换”的数据结构和函数声明，不包含任何控制策略。
 * 在当前工程里，它被以下模块复用：
 * - foc_current_loop：把三相电流变到 dq，再把 dq 电压变回 alpha/beta；
 * - foc_observer：把三相电流变成 alpha/beta 后送入磁链观测器；
 * - app_foc：对齐阶段需要把 dq 电压命令转换为 alpha/beta 电压命令。
 *
 * 坐标约定：
 * - alpha/beta 是静止坐标系；
 * - d/q 是跟随电角度 theta_rad 旋转的坐标系；
 * - theta_rad 必须是电角度，不是机械角度。
 */

typedef struct
{
  /* alpha 轴分量，通常与 U 相轴重合。 */
  float alpha;
  /* beta 轴分量，与 alpha 轴正交。 */
  float beta;
} ClarkePark_AlphaBeta_t;

typedef struct
{
  /* d 轴分量，通常代表励磁方向。 */
  float d;
  /* q 轴分量，通常代表转矩方向。 */
  float q;
} ClarkePark_DQ_t;

/* 三相 -> alpha/beta。
 * ia/ib/ic 理论上满足 ia+ib+ic=0；当前实现主要用 ia/ib，ic 保留接口一致性。
 */
void ClarkePark_Clarke(float ia, float ib, float ic, ClarkePark_AlphaBeta_t *out);

/* alpha/beta -> d/q。
 * theta_rad 为控制角或观测角，方向必须和 InvPark 保持一致。
 */
void ClarkePark_Park(const ClarkePark_AlphaBeta_t *in, float theta_rad, ClarkePark_DQ_t *out);

/* d/q -> alpha/beta。
 * 电流环输出的 vd/vq 最终会通过这里变成 SVPWM 输入 valpha/vbeta。
 */
void ClarkePark_InvPark(const ClarkePark_DQ_t *in, float theta_rad, ClarkePark_AlphaBeta_t *out);

/* alpha/beta -> 三相。
 * 当前主控主要使用 alpha/beta 直接进 SVPWM，本函数保留给调试或建模使用。
 */
void ClarkePark_InvClarke(const ClarkePark_AlphaBeta_t *in, float *va, float *vb, float *vc);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SENSORLESS_CLARKE_PARK_H */
