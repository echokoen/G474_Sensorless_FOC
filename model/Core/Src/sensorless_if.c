#include "sensorless_if.h"
#include <math.h>

/* 旧版无感接口封装。
 *
 * 当前主链路已经转向 App/Control + FluxObserver_PLL，
 * 这里保留仅用于兼容历史实验和对照测试。
 */
static SensorlessObserver_t s_obs;
static float s_last_valpha = 0.0f;
static float s_last_vbeta = 0.0f;
static float s_last_ialpha = 0.0f;
static float s_last_ibeta = 0.0f;
static float s_last_vbus = 0.0f;
static float s_theta_est_rad = 0.0f;
static float s_speed_est_rad_s = 0.0f;
static float s_angle_err_deg = 0.0f;
static uint8_t s_reliable = 0u;
static float s_flux_alpha = 0.0f;
static float s_flux_beta = 0.0f;
static float s_bemf_alpha = 0.0f;
static float s_bemf_beta = 0.0f;

void SensorlessIf_Init(void)
{
  SensorlessObserver_Init(&s_obs);
  SensorlessIf_Reset();
}

void SensorlessIf_Reset(void)
{
  SensorlessObserver_Reset(&s_obs);
  s_last_valpha = 0.0f;
  s_last_vbeta = 0.0f;
  s_last_ialpha = 0.0f;
  s_last_ibeta = 0.0f;
  s_last_vbus = 0.0f;
  s_theta_est_rad = 0.0f;
  s_speed_est_rad_s = 0.0f;
  s_angle_err_deg = 0.0f;
  s_reliable = 0u;
  s_flux_alpha = 0.0f;
  s_flux_beta = 0.0f;
  s_bemf_alpha = 0.0f;
  s_bemf_beta = 0.0f;
}

void SensorlessIf_Update(float valpha, float vbeta, float ialpha, float ibeta, float vbus)
{
  s_obs.Valpha = s_last_valpha;
  s_obs.Vbeta = s_last_vbeta;
  s_obs.Ialpha = ialpha;
  s_obs.Ibeta = ibeta;
  s_obs.Vbus = vbus;
  s_last_valpha = valpha;
  s_last_vbeta = vbeta;
  s_last_ialpha = ialpha;
  s_last_ibeta = ibeta;
  s_last_vbus = vbus;
  SensorlessObserver_Update(&s_obs);
  s_theta_est_rad = SensorlessObserver_GetEstimatedThetaRad(&s_obs);
  s_speed_est_rad_s = SensorlessObserver_GetEstimatedSpeedRadPerSec(&s_obs);
  s_flux_alpha = s_obs.FluxAlpha;
  s_flux_beta = s_obs.FluxBeta;
  s_bemf_alpha = SensorlessObserver_GetEstimatedBemfAlpha(&s_obs);
  s_bemf_beta = SensorlessObserver_GetEstimatedBemfBeta(&s_obs);
  s_reliable = SensorlessObserver_IsConverged(&s_obs) ? 1u : 0u;
  s_angle_err_deg = s_obs.AngleErrRad * 57.29577951308232f;
}

float SensorlessIf_GetThetaEstRad(void) { return s_theta_est_rad; }
float SensorlessIf_GetSpeedEstRadPerSec(void) { return s_speed_est_rad_s; }
float SensorlessIf_GetAngleErrDeg(void) { return s_angle_err_deg; }
uint8_t SensorlessIf_IsReliable(void) { return s_reliable; }

void SensorlessIf_GetInputs(float *valpha, float *vbeta, float *ialpha, float *ibeta, float *vbus)
{
  if (valpha) *valpha = s_obs.Valpha;
  if (vbeta) *vbeta = s_obs.Vbeta;
  if (ialpha) *ialpha = s_obs.Ialpha;
  if (ibeta) *ibeta = s_obs.Ibeta;
  if (vbus) *vbus = s_obs.Vbus;
}

void SensorlessIf_GetFlux(float *flux_alpha, float *flux_beta)
{
  if (flux_alpha) *flux_alpha = s_flux_alpha;
  if (flux_beta) *flux_beta = s_flux_beta;
}

void SensorlessIf_GetBEMF(float *bemf_alpha, float *bemf_beta)
{
  if (bemf_alpha) *bemf_alpha = s_bemf_alpha;
  if (bemf_beta) *bemf_beta = s_bemf_beta;
}
