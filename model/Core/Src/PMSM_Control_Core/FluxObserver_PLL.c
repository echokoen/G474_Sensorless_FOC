#include "PMSM_Control_Core/FluxObserver_PLL.h"
#include <math.h>

/* 磁链观测器 + PLL 实现。
 *
 * 这份实现按“先磁链、后 PLL”的顺序拆开：
 * - 磁链观测器负责把电压、电流和电机参数融合起来，估算磁链矢量；
 * - PLL 负责从磁链矢量里提取连续、平滑的角度和速度；
 * - 两者分开后，调试时更容易看出问题到底出在哪一层。
 *
 * 一个实用的判断方式是：
 * - 如果磁链本身就不转，优先查电流、电压和参数；
 * - 如果磁链能转但 PLL 不跟，优先查 PLL 增益和相位误差。
 */

struct FluxObserver_PLL_t fluxObserver_pll_est = {0};

/* 内部状态。
 *
 * 这些状态不放到输出结构体里，是因为上层只需要输入/输出量；
 * 积分器和 PLL 状态属于算法内部记忆。
 */
static float s_flux_s_alpha = 0.0f;      /* 定子磁链 alpha 积分状态。 */
static float s_flux_s_beta = 0.0f;       /* 定子磁链 beta 积分状态。 */
static float s_theta_rad = 0.0f;         /* PLL 当前估算角度。 */
static float s_speed_rad_s = 0.0f;       /* PLL 未滤波速度。 */
static float s_speed_filt_rad_s = 0.0f;  /* PLL 滤波后速度，对外输出。 */
static float s_pll_integral = 0.0f;      /* PLL 积分项，决定稳态速度估计。 */

static float clampf_local(float value, float min_value, float max_value)
{
    /* 本文件内部使用的轻量限幅，避免依赖外层工具函数。 */
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

/* 重置观测器状态。
 *
 * 为什么要单独重置内部状态：
 * - 磁链积分器会记住上一次运行的残留；
 * - PLL 的积分项也会记住上一次的角速度偏差；
 * - 如果不清空，重新启动时很容易带着旧状态“跑偏”。
 *
 * 这里做的事情相当于给 observer “清脑子”。
 */
void FluxObserver_PLL_reset(struct FluxObserver_PLL_t *obs)
{
    if (obs == NULL)
    {
        return;
    }

    obs->Valpha_I = 0.0f;
    obs->Vbeta_I = 0.0f;
    obs->Ialpha_I = 0.0f;
    obs->Ibeta_I = 0.0f;
    obs->Flux_alpha_O = 0.0f;
    obs->Flux_beta_O = 0.0f;
    obs->Espeed_O = 0.0f;
    obs->Etheta_O = 0.0f;
    obs->pll_err = 0.0f;
    obs->pll_integral = 0.0f;
    obs->pll_speed_raw_rad_s = 0.0f;
    obs->pll_speed_filt_rad_s = 0.0f;
    obs->flux_mag = 0.0f;
    obs->speed_limit_hit = 0u;
    obs->integral_limit_hit = 0u;

    s_flux_s_alpha = FOC_FLUX_WB;
    s_flux_s_beta = 0.0f;
    s_theta_rad = 0.0f;
    s_speed_rad_s = 0.0f;
    s_speed_filt_rad_s = 0.0f;
    s_pll_integral = 0.0f;
}

/* 磁链更新。
 *
 * 这里采用的是最简形式的磁链积分 + 非线性修正：
 * - `u - R*i` 是电压减去铜耗后的有效积分项；
 * - `Ls*i` 用来从定子磁链里扣掉电感分量，得到更接近转子磁链的估计；
 * - `lambda` 和 `k` 控制非线性修正强度与磁链约束形状。
 *
 * 如果这个步骤的输入是对的，`Flux_alpha_O / Flux_beta_O` 至少应该表现出“有方向、有幅值”的旋转趋势。
 */
static void FluxObserver_update(struct FluxObserver_PLL_t *obs)
{
    const float Ts = FOC_TS_SEC;
    const float Rs = FOC_RS_OHM;
    const float Ls = FOC_LS_H;
    const float flux_e = FOC_FLUX_WB;

    const float ualpha = obs->Valpha_I;
    const float ubeta = obs->Vbeta_I;
    const float ialpha = obs->Ialpha_I;
    const float ibeta = obs->Ibeta_I;

    /* 定子磁链积分：
     * psi_s = integral(u_s - R_s * i_s) dt
     */
    s_flux_s_alpha += Ts * (ualpha - Rs * ialpha);
    s_flux_s_beta += Ts * (ubeta - Rs * ibeta);

    /* 从定子磁链中扣除电感磁链，得到近似转子磁链。 */
    float flux_r_alpha = s_flux_s_alpha - Ls * ialpha;
    float flux_r_beta = s_flux_s_beta - Ls * ibeta;

    /* 对磁链幅值做上限约束。
     *
     * 这里不要每拍都把磁链硬拉到 FOC_FLUX_WB，否则幅值噪声会被转换成
     * 角度/速度扰动。只有磁链明显超过合理范围时，才按比例缩回。
     */
    const float flux_mag = sqrtf((flux_r_alpha * flux_r_alpha) + (flux_r_beta * flux_r_beta));
    const float flux_limit = flux_e * FOC_FLUX_LIMIT_RATIO;
    if ((flux_mag > flux_limit) && (flux_mag > 1.0e-6f))
    {
        const float scale = flux_limit / flux_mag;
        flux_r_alpha *= scale;
        flux_r_beta *= scale;
        s_flux_s_alpha = flux_r_alpha + Ls * ialpha;
        s_flux_s_beta = flux_r_beta + Ls * ibeta;
    }

    /* 把当前估算出的磁链向量回写到输出结构里，供上层打印和分析。 */
    obs->Flux_alpha_O = flux_r_alpha;
    obs->Flux_beta_O = flux_r_beta;
    obs->flux_mag = sqrtf((flux_r_alpha * flux_r_alpha) + (flux_r_beta * flux_r_beta));
}

/* PLL 更新。
 *
 * 通过磁链向量与当前角度的正交误差来驱动速度和角度积分：
 * - 误差为正，说明当前角度还没追上磁链；
 * - 误差为负，说明角度超前；
 * - `kp` 负责快速纠偏；
 * - `ki` 负责消除稳态误差。
 *
 * 所以 PLL 这里本质上是在问：
 * “我现在猜的角度，和磁链真实方向之间差多少？”
 */
static void PLL_update(struct FluxObserver_PLL_t *obs)
{
    const float Ts = FOC_TS_SEC;
    const float kp = FOC_OBS_PLL_KP;
    const float ki = FOC_OBS_PLL_KI;
    const float two_pi = 6.2831853071795864769f;
    const float speed_alpha = clampf_local(FOC_OBS_SPEED_FILTER_ALPHA, 0.0f, 1.0f);

    const float sin_t = sinf(s_theta_rad);
    const float cos_t = cosf(s_theta_rad);
    const float flux_mag = sqrtf((obs->Flux_alpha_O * obs->Flux_alpha_O) + (obs->Flux_beta_O * obs->Flux_beta_O));

    /* 正交误差：
     * 当 `s_theta_rad` 正好对齐磁链时，这个误差会接近 0。
     * 偏差越大，说明角度估计和磁链方向差得越远。
     */
    float err = obs->Flux_beta_O * cos_t - obs->Flux_alpha_O * sin_t;
    if (flux_mag > 1.0e-6f)
    {
        err /= flux_mag;
    }
    s_pll_integral += ki * err * Ts;
    s_pll_integral = clampf_local(s_pll_integral,
                                  -FOC_OBS_PLL_INTEGRAL_LIMIT,
                                  FOC_OBS_PLL_INTEGRAL_LIMIT);

    s_speed_rad_s = s_pll_integral + kp * err;
    s_speed_rad_s = clampf_local(s_speed_rad_s,
                                 -FOC_OBS_SPEED_LIMIT_RAD_S,
                                 FOC_OBS_SPEED_LIMIT_RAD_S);
    s_speed_filt_rad_s += speed_alpha * (s_speed_rad_s - s_speed_filt_rad_s);
    s_theta_rad += s_speed_rad_s * Ts;

    while (s_theta_rad >= two_pi) s_theta_rad -= two_pi;
    while (s_theta_rad < 0.0f) s_theta_rad += two_pi;

    /* 把 PLL 的速度和角度写回输出。
     * 这里的角度已经被包回 [0, 2pi)，方便上层直接画图。
     */
    obs->pll_err = err;
    obs->pll_integral = s_pll_integral;
    obs->pll_speed_raw_rad_s = s_speed_rad_s;
    obs->pll_speed_filt_rad_s = s_speed_filt_rad_s;
    obs->speed_limit_hit = (fabsf(s_speed_rad_s) >= (FOC_OBS_SPEED_LIMIT_RAD_S * 0.98f)) ? 1u : 0u;
    obs->integral_limit_hit = (fabsf(s_pll_integral) >= (FOC_OBS_PLL_INTEGRAL_LIMIT * 0.98f)) ? 1u : 0u;
    obs->Espeed_O = s_speed_filt_rad_s;
    obs->Etheta_O = s_theta_rad;
}

/* 对外的一拍更新入口。
 * 调用顺序固定为：先更新磁链，再用磁链结果更新 PLL。
 *
 * 把这一步封装起来的意义是：
 * - 上层只需要保证输入已填好；
 * - 不需要关心内部先后顺序；
 * - 也不容易在别处把磁链和 PLL 的更新顺序写乱。
 */
void FluxObserver_PLL_update(struct FluxObserver_PLL_t *obs)
{
    if (obs == NULL)
    {
        return;
    }

    FluxObserver_update(obs);
    PLL_update(obs);
}
