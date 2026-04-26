#ifndef FOC_SENSORLESS_FluxObserver_PLL_H
#define FOC_SENSORLESS_FluxObserver_PLL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "foc_config.h"

/* 磁链观测器 + PLL 的输入输出接口。
 *
 * 这一层保持为 donor 算法内核，当前工程只负责喂数据和取结果。
 */

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
    float Valpha_I;       /* 输入：alpha 轴电压，单位 V。 */
    float Vbeta_I;        /* 输入：beta 轴电压，单位 V。 */
    float Ialpha_I;       /* 输入：alpha 轴电流，单位 A。 */
    float Ibeta_I;        /* 输入：beta 轴电流，单位 A。 */

    float Flux_alpha_O;   /* 输出：alpha 轴转子磁链估计，单位 Wb。 */
    float Flux_beta_O;    /* 输出：beta 轴转子磁链估计，单位 Wb。 */
    float Espeed_O;       /* 输出：PLL 滤波后的电角速度，单位 rad/s。 */
    float Etheta_O;       /* 输出：PLL 估算电角度，范围 [0, 2pi)。 */

    float pll_err;              /* 调试：PLL 归一化相位误差。 */
    float pll_integral;         /* 调试：PLL 积分项，单位 rad/s。 */
    float pll_speed_raw_rad_s;  /* 调试：PLL 未滤波电角速度，单位 rad/s。 */
    float pll_speed_filt_rad_s; /* 调试：PLL 滤波后电角速度，单位 rad/s。 */
    float flux_mag;             /* 调试：转子磁链幅值，单位 Wb。 */
    uint8_t speed_limit_hit;    /* 调试：速度限幅是否接近打满。 */
    uint8_t integral_limit_hit; /* 调试：积分限幅是否接近打满。 */
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
