#include "PMSM_Control_Core/FluxObserver_PLL.h"
#include <math.h>

/* 第二阶段：磁链观测器 + PLL 后台运行，不接管主控制角。
 * 为什么这样做：先让观测器跟踪开环旋转轨迹，
 * 再验证角度和速度估算是否足够稳定。 */

struct FluxObserver_PLL_t fluxObserver_pll_est = {0};

static float s_flux_s_alpha = 0.0f;
static float s_flux_s_beta = 0.0f;
static float s_theta_rad = 0.0f;
static float s_speed_rad_s = 0.0f;
static float s_pll_integral = 0.0f;

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

static void FluxObserver_update(struct FluxObserver_PLL_t *obs)
{
    const float Ts = FOC_TS_SEC;
    const float Rs = FOC_RS_OHM;
    const float Ls = FOC_LS_H;
    const float flux_e = FOC_FLUX_WB;
    const float lambda = 600.0f;
    const float k = 1.0f;

    const float ualpha = obs->Valpha_I;
    const float ubeta = obs->Vbeta_I;
    const float ialpha = obs->Ialpha_I;
    const float ibeta = obs->Ibeta_I;

    const float flux_r_alpha = s_flux_s_alpha - Ls * ialpha;
    const float flux_r_beta = s_flux_s_beta - Ls * ibeta;
    const float flux_err = (flux_e * flux_e) - (flux_r_alpha * flux_r_alpha) - (flux_r_beta * flux_r_beta);

    s_flux_s_alpha = s_flux_s_alpha + Ts * (
        ualpha - Rs * ialpha + (lambda * 0.5f) * (flux_r_alpha - k * flux_r_beta) * flux_err);
    s_flux_s_beta = s_flux_s_beta + Ts * (
        ubeta - Rs * ibeta + (lambda * 0.5f) * (flux_r_beta + k * flux_r_alpha) * flux_err);

    obs->Flux_alpha_O = flux_r_alpha;
    obs->Flux_beta_O = flux_r_beta;
}

static void PLL_update(struct FluxObserver_PLL_t *obs)
{
    const float Ts = FOC_TS_SEC;
    const float kp = 120.0f;
    const float ki = 8000.0f;
    const float two_pi = 6.2831853071795864769f;

    const float sin_t = sinf(s_theta_rad);
    const float cos_t = cosf(s_theta_rad);

    const float err = obs->Flux_beta_O * cos_t - obs->Flux_alpha_O * sin_t;
    s_pll_integral += ki * err * Ts;
    s_speed_rad_s = s_pll_integral + kp * err;
    s_theta_rad += s_speed_rad_s * Ts;

    while (s_theta_rad >= two_pi) s_theta_rad -= two_pi;
    while (s_theta_rad < 0.0f) s_theta_rad += two_pi;

    obs->Espeed_O = s_speed_rad_s;
    obs->Etheta_O = s_theta_rad;
}

void FluxObserver_PLL_update(struct FluxObserver_PLL_t *obs)
{
    if (obs == NULL)
    {
        return;
    }

    FluxObserver_update(obs);
    PLL_update(obs);
}
