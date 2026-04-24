#ifndef __FOC_DEADTIME_H__
#define __FOC_DEADTIME_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "foc_types.h"

/* 死区补偿运行数据。
 *
 * 死区会让实际相电压随电流方向产生偏差。
 * 本模块根据三相电流符号估算一个补偿电压，再转换回 alpha/beta。
 */
typedef struct
{
  float k_used;      /* 本拍实际使用的补偿系数。 */
  float vdt_u;       /* U 相死区补偿电压。 */
  float vdt_v;       /* V 相死区补偿电压。 */
  float vdt_w;       /* W 相死区补偿电压。 */
  float valpha_out;  /* 补偿后的 alpha 电压。 */
  float vbeta_out;   /* 补偿后的 beta 电压。 */
} FOC_DeadtimeComp_t;

void FOC_DeadtimeComp_Init(FOC_DeadtimeComp_t *dt);
void FOC_DeadtimeComp_Reset(FOC_DeadtimeComp_t *dt);

/* 对 alpha/beta 电压指令做死区补偿。
 *
 * 输入：
 * - vbus_v：母线电压，用于把补偿系数换算成实际电压；
 * - iu/iv/iw：三相电流，用于判断补偿方向；
 * - valpha_in/vbeta_in：原始电压指令。
 *
 * 输出：
 * - valpha_out/vbeta_out：补偿后的电压指令。
 */
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
