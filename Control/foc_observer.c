#include "foc_observer.h"
#include "foc_config.h"
#include "foc_math.h"
#include <math.h>

/* 观测器封装实现。
 *
 * 负责驱动 FluxObserver_PLL 并缓存角度和速度。
 * 这个文件是“工程适配层”，不是观测器数学本体：
 * - 电压输入来自电流环最终输出的 valpha/vbeta；
 * - 电流输入来自采样模块重构后的 iu/iv；
 * - theta 补偿在这里完成；
 * - observer 接管判定由 foc_switchover.c 完成。
 */
void FOC_ObserverInit(FOC_ObserverData_t *observer)
{
  if (observer == 0)
  {
    return;
  }

  FluxObserver_PLL_reset(&observer->flux_obs);
  observer->theta_rad = 0.0f;
  observer->speed_rad_s = 0.0f;
}

void FOC_ObserverReset(FOC_ObserverData_t *observer)
{
  FOC_ObserverInit(observer);
}

void FOC_ObserverUpdate(FOC_ObserverData_t *observer,
                        const FOC_SamplingData_t *sampling,
                        float valpha,
                        float vbeta)
{
  if ((observer == 0) || (sampling == 0))
  {
    return;
  }

  observer->flux_obs.Valpha_I = valpha;
  observer->flux_obs.Vbeta_I = vbeta;

  /* Clarke 变换在三相电流和为 0 的前提下可简化为：
   * Ialpha = iu
   * Ibeta  = (iu + 2*iv) / sqrt(3)
   * 这里不用再引入完整 Clarke 结构体，是为了减少 observer 路径开销。
   */
  observer->flux_obs.Ialpha_I = sampling->iu_a;
  observer->flux_obs.Ibeta_I = (sampling->iu_a + 2.0f * sampling->iv_a) * 0.57735026919f;

  /* 先更新磁链与 PLL，再把工程补偿角加到 observer 原始角上。 */
  FluxObserver_PLL_update(&observer->flux_obs);
  observer->theta_rad = FOC_MathWrap2Pi(observer->flux_obs.Etheta_O + FOC_OBS_THETA_COMP_RAD);
  observer->speed_rad_s = observer->flux_obs.Espeed_O;
}

float FOC_ObserverGetFluxThetaRad(const FOC_ObserverData_t *observer)
{
  /* 直接用磁链 alpha/beta 求原始磁链方向，主要用于调试对照 PLL 角。 */
  return (observer != 0) ? atan2f(observer->flux_obs.Flux_beta_O, observer->flux_obs.Flux_alpha_O) : 0.0f;
}

float FOC_ObserverGetThetaRad(const FOC_ObserverData_t *observer)
{
  return (observer != 0) ? observer->theta_rad : 0.0f;
}

float FOC_ObserverGetSpeedRadPerSec(const FOC_ObserverData_t *observer)
{
  return (observer != 0) ? observer->speed_rad_s : 0.0f;
}

float FOC_ObserverGetAngleErrDeg(const FOC_ObserverData_t *observer)
{
  (void)observer;
  return 0.0f;
}

uint8_t FOC_ObserverIsLocked(const FOC_ObserverData_t *observer)
{
  (void)observer;
  return 0u;
}

void FOC_ObserverGetInputs(const FOC_ObserverData_t *observer,
                           float *valpha,
                           float *vbeta,
                           float *ialpha,
                           float *ibeta)
{
  if (observer == 0)
  {
    return;
  }

  if (valpha) *valpha = observer->flux_obs.Valpha_I;
  if (vbeta) *vbeta = observer->flux_obs.Vbeta_I;
  if (ialpha) *ialpha = observer->flux_obs.Ialpha_I;
  if (ibeta) *ibeta = observer->flux_obs.Ibeta_I;
}
