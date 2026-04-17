#ifndef FOC_SENSORLESS_FluxObserver_PLL_H
#define FOC_SENSORLESS_FluxObserver_PLL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "foc_config.h"

/* 磁链观测器 + PLL 的数据交换结构。
 *
 * 输入：
 * - `Valpha_I`, `Vbeta_I`：当前拍用于观测器积分的电压矢量。
 * - `Ialpha_I`, `Ibeta_I`：当前拍测得的定子电流矢量。
 *
 * 输出：
 * - `Flux_alpha_O`, `Flux_beta_O`：估算得到的磁链矢量。
 * - `Espeed_O`：PLL 给出的电角速度估计值。
 * - `Etheta_O`：PLL 给出的电角度估计值。
 */
struct FluxObserver_PLL_t {
    float Valpha_I;
    float Vbeta_I;
    float Ialpha_I;
    float Ibeta_I;

    float Flux_alpha_O;
    float Flux_beta_O;
    float Espeed_O;
    float Etheta_O;
};

extern struct FluxObserver_PLL_t fluxObserver_pll_est;

/* 清空内部状态，适合上电、重新启动或调试重置时调用。 */
void FluxObserver_PLL_reset(struct FluxObserver_PLL_t *obs);

/* 执行一次磁链观测器 + PLL 更新。
 * 该函数假设输入已经在调用前填好，且调用频率与 `FOC_TS_SEC` 一致。
 */
void FluxObserver_PLL_update(struct FluxObserver_PLL_t *obs);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SENSORLESS_FluxObserver_PLL_H */
