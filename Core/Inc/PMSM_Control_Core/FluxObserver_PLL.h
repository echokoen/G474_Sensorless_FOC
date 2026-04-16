#ifndef FOC_SENSORLESS_FluxObserver_PLL_H
#define FOC_SENSORLESS_FluxObserver_PLL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "foc_config.h"

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

void FluxObserver_PLL_reset(struct FluxObserver_PLL_t *obs);
void FluxObserver_PLL_update(struct FluxObserver_PLL_t *obs);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SENSORLESS_FluxObserver_PLL_H */
