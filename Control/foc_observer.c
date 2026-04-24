#include "foc_observer.h"
#include "foc_config.h"
#include "foc_math.h"
#include <math.h>

/* 观测器封装实现。
 *
 * 负责驱动 FluxObserver_PLL 并缓存角度、速度、角差和锁定状态。
 * 这个文件是“工程适配层”，不是观测器数学本体：
 * - 电压输入来自电流环最终输出的 valpha/vbeta；
 * - 电流输入来自采样模块重构后的 iu/iv；
 * - theta 补偿和 locked 判定在这里完成。
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
  observer->angle_err_deg = 0.0f;
  observer->locked = 0u;
  observer->hold_ticks = 0u;
}

void FOC_ObserverReset(FOC_ObserverData_t *observer)
{
  FOC_ObserverInit(observer);
}

void FOC_ObserverUpdate(FOC_ObserverData_t *observer,
                        const FOC_SamplingData_t *sampling,
                        float valpha,
                        float vbeta,
                        float theta_open_rad,
                        float mech_freq_hz)
{
  float angle_err_rad;
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

  /* angle_err 用于判断开环角和 observer 角是否已经足够接近。
   * WrapPi 后的误差范围是 [-pi, pi]，转成角度便于串口观察。
   */
  angle_err_rad = FOC_MathWrapPi(theta_open_rad - observer->theta_rad);
  observer->angle_err_deg = FOC_MathRadToDeg(angle_err_rad);

  /* observer 锁定判定：
   * - 低速时磁链观测容易不可靠，所以先要求机械频率达到门限；
   * - 角差必须落在允许范围内；
   * - 条件要连续满足一段时间，避免瞬间误判。
   */
  if ((mech_freq_hz >= FOC_OBS_ENABLE_FREQ_HZ) && (fabsf(observer->angle_err_deg) < FOC_OBS_LOCK_ERR_DEG))
  {
    if (observer->hold_ticks < (uint32_t)(FOC_OBS_LOCK_HOLD_MS / (FOC_TS_SEC * 1000.0f)))
    {
      observer->hold_ticks++;
    }
    else
    {
      observer->locked = 1u;
    }
  }
  else
  {
    observer->hold_ticks = 0u;
    observer->locked = 0u;
  }
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
  return (observer != 0) ? observer->angle_err_deg : 0.0f;
}

uint8_t FOC_ObserverIsLocked(const FOC_ObserverData_t *observer)
{
  return (observer != 0) ? observer->locked : 0u;
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
