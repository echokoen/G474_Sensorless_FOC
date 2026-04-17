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

static float s_flux_s_alpha = 0.0f;
static float s_flux_s_beta = 0.0f;
static float s_theta_rad = 0.0f;
static float s_speed_rad_s = 0.0f;
static float s_pll_integral = 0.0f;

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

    s_flux_s_alpha = FOC_FLUX_WB;
    s_flux_s_beta = 0.0f;
    s_theta_rad = 0.0f;
    s_speed_rad_s = 0.0f;
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

    s_flux_s_alpha += Ts * (ualpha - Rs * ialpha);
    s_flux_s_beta += Ts * (ubeta - Rs * ibeta);

    float flux_r_alpha = s_flux_s_alpha - Ls * ialpha;
    float flux_r_beta = s_flux_s_beta - Ls * ibeta;

    /* 对磁链幅值做软限幅。
     *
     * 之前的非线性修正项在当前小电压开环下容易把磁链拉到固定方向，
     * 表现就是 theta_flux 长时间卡在 +/-pi 附近。这里先用更朴素的
     * 电压模型积分，再只约束幅值，不强行改变磁链方向。
     */
    const float flux_mag = sqrtf((flux_r_alpha * flux_r_alpha) + (flux_r_beta * flux_r_beta));
    if (flux_mag > 1.0e-6f)
    {
        const float scale = flux_e / flux_mag;
        flux_r_alpha *= scale;
        flux_r_beta *= scale;
        s_flux_s_alpha = flux_r_alpha + Ls * ialpha;
        s_flux_s_beta = flux_r_beta + Ls * ibeta;
    }

    /* 把当前估算出的磁链向量回写到输出结构里，供上层打印和分析。 */
    obs->Flux_alpha_O = flux_r_alpha;
    obs->Flux_beta_O = flux_r_beta;
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
    const float kp = 250.0f;
    const float ki = 12000.0f;
    const float two_pi = 6.2831853071795864769f;

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
    s_speed_rad_s = s_pll_integral + kp * err;
    s_theta_rad += s_speed_rad_s * Ts;

    while (s_theta_rad >= two_pi) s_theta_rad -= two_pi;
    while (s_theta_rad < 0.0f) s_theta_rad += two_pi;

    /* 把 PLL 的速度和角度写回输出。
     * 这里的角度已经被包回 [0, 2pi)，方便上层直接画图。
     */
    obs->Espeed_O = s_speed_rad_s;
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
