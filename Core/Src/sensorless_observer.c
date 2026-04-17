#include "sensorless_observer.h"
#include "foc_config.h"
#include <math.h>

static float wrap_2pi(float angle)
{
  const float two_pi = 6.2831853071795864769f;
  while (angle >= two_pi) angle -= two_pi;
  while (angle < 0.0f) angle += two_pi;
  return angle;
}

static float wrap_pi(float angle)
{
  const float pi = 3.14159265358979323846f;
  const float two_pi = 6.2831853071795864769f;
  while (angle > pi) angle -= two_pi;
  while (angle < -pi) angle += two_pi;
  return angle;
}

static void sensorless_init_speed_buffer(SensorlessObserver_t *obs)
{
  for (uint8_t i = 0u; i < obs->SpeedBufferSize; i++)
  {
    obs->SpeedBuffer[i] = 0.0f;
  }
  obs->SpeedBufferIndex = 0u;
  obs->SpeedBufferSum = 0.0f;
  obs->SpeedAverage = 0.0f;
  obs->SpeedBufferOldestEl = 0.0f;
}

static void sensorless_store_rotor_speed(SensorlessObserver_t *obs, int16_t rotor_speed_dpp)
{
  uint8_t idx = obs->SpeedBufferIndex;
  idx++;
  if (idx >= obs->SpeedBufferSize)
  {
    idx = 0u;
  }
  obs->SpeedBufferOldestEl = obs->SpeedBuffer[idx];
  obs->SpeedBufferSum -= obs->SpeedBuffer[idx];
  obs->SpeedBuffer[idx] = (float)rotor_speed_dpp;
  obs->SpeedBufferSum += (float)rotor_speed_dpp;
  obs->SpeedBufferIndex = idx;
  obs->SpeedAverage = (obs->SpeedBufferSize == 0u) ? (float)rotor_speed_dpp : (obs->SpeedBufferSum / (float)obs->SpeedBufferSize);
}

void SensorlessObserver_Init(SensorlessObserver_t *obs)
{
  if (obs == NULL)
  {
    return;
  }

  obs->SpeedBufferSize = SENSORLESS_SPEED_BUFFER_SIZE;
  obs->SpeedBufferSizeDpp = SENSORLESS_SPEED_BUFFER_SIZE;
  obs->EnableDualCheck = 1u;
  obs->ForcedDirection = 0;
  SensorlessObserver_Reset(obs);
}

void SensorlessObserver_Reset(SensorlessObserver_t *obs)
{
  if (obs == NULL)
  {
    return;
  }

  obs->Vbus = 0.0f;
  obs->Valpha = 0.0f;
  obs->Vbeta = 0.0f;
  obs->Ialpha = 0.0f;
  obs->Ibeta = 0.0f;
  obs->Ialfa_est = 0.0f;
  obs->Ibeta_est = 0.0f;
  obs->Bemf_alfa_est = 0.0f;
  obs->Bemf_beta_est = 0.0f;
  obs->FluxAlpha = 0.0f;
  obs->FluxBeta = 0.0f;
  obs->ThetaEstRad = 0.0f;
  obs->SpeedEstRadPerSec = 0.0f;
  obs->AngleErrRad = 0.0f;
  obs->SpeedBufferIndex = 0u;
  obs->SpeedBufferSum = 0.0f;
  obs->SpeedAverage = 0.0f;
  obs->SpeedBufferOldestEl = 0.0f;
  obs->IsSpeedReliable = 0u;
  obs->IsBemfConsistent = 0u;
  obs->IsConverged = 0u;
  obs->ReliabilityCounter = 0u;
  obs->ConsistencyCounter = 0u;
  obs->PllIntegral = 0.0f;
  obs->FluxMagSq = 0.0f;
  obs->BemfMagSq = 0.0f;
  obs->hElSpeedDpp = 0;
  obs->hElAngle = 0;
  obs->ForceConvergency = 0u;
  obs->ForceConvergency2 = 0u;
  sensorless_init_speed_buffer(obs);
}

void SensorlessObserver_Update(SensorlessObserver_t *obs)
{
  if (obs == NULL)
  {
    return;
  }

  const float Rs = FOC_RS_OHM;
  const float Ls = FOC_LS_H;
  const float flux_e = FOC_FLUX_WB;
  const float lambda = 600.0f;
  const float k = 1.0f;

  const float flux_r_alpha = obs->FluxAlpha - Ls * obs->Ialpha;
  const float flux_r_beta = obs->FluxBeta - Ls * obs->Ibeta;
  const float flux_err = (flux_e * flux_e) - (flux_r_alpha * flux_r_alpha) - (flux_r_beta * flux_r_beta);

  obs->FluxAlpha = obs->FluxAlpha + FOC_TS_SEC * (obs->Valpha - Rs * obs->Ialpha + (lambda * 0.5f) * (flux_r_alpha - k * flux_r_beta) * flux_err);
  obs->FluxBeta  = obs->FluxBeta  + FOC_TS_SEC * (obs->Vbeta  - Rs * obs->Ibeta  + (lambda * 0.5f) * (flux_r_beta + k * flux_r_alpha) * flux_err);

  obs->Bemf_alfa_est = flux_r_alpha;
  obs->Bemf_beta_est = flux_r_beta;
  obs->BemfMagSq = (obs->Bemf_alfa_est * obs->Bemf_alfa_est) + (obs->Bemf_beta_est * obs->Bemf_beta_est);
  obs->Ialfa_est = obs->Ialpha;
  obs->Ibeta_est = obs->Ibeta;
  obs->FluxMagSq = (obs->FluxAlpha * obs->FluxAlpha) + (obs->FluxBeta * obs->FluxBeta);

  {
    const float sin_t = sinf(obs->ThetaEstRad);
    const float cos_t = cosf(obs->ThetaEstRad);
    const float err = (obs->Bemf_beta_est * cos_t) - (obs->Bemf_alfa_est * sin_t);
    const float kp = 120.0f;
    const float ki = 8000.0f;
    obs->PllIntegral += ki * err * FOC_TS_SEC;
    obs->SpeedEstRadPerSec = obs->PllIntegral + (kp * err);
    obs->ThetaEstRad = wrap_2pi(obs->ThetaEstRad + obs->SpeedEstRadPerSec * FOC_TS_SEC);
    obs->AngleErrRad = wrap_pi(atan2f(obs->Bemf_beta_est, obs->Bemf_alfa_est) - obs->ThetaEstRad);
  }

  obs->hElSpeedDpp = (int16_t)(obs->SpeedEstRadPerSec * 57.2957795f);
  obs->hElAngle = (int16_t)(obs->ThetaEstRad * 57.2957795f);
  sensorless_store_rotor_speed(obs, obs->hElSpeedDpp);

  obs->IsSpeedReliable = (uint8_t)((fabsf(obs->AngleErrRad) < 0.35f) ? 1u : 0u);
  obs->IsBemfConsistent = (uint8_t)((obs->BemfMagSq > 1.0e-8f) ? 1u : 0u);

  if ((obs->ForceConvergency != 0u) || (obs->ForceConvergency2 != 0u))
  {
    obs->IsConverged = 1u;
  }
  else if ((obs->IsSpeedReliable != 0u) && (obs->IsBemfConsistent != 0u))
  {
    if (obs->ConsistencyCounter < 255u) obs->ConsistencyCounter++;
    if (obs->ConsistencyCounter > 10u)
    {
      obs->IsConverged = 1u;
    }
  }
  else
  {
    obs->ConsistencyCounter = 0u;
    obs->IsConverged = 0u;
  }
}

bool SensorlessObserver_IsConverged(const SensorlessObserver_t *obs)
{
  return (obs == NULL) ? false : (obs->IsConverged != 0u);
}

float SensorlessObserver_GetEstimatedBemfAlpha(const SensorlessObserver_t *obs)
{
  return (obs == NULL) ? 0.0f : obs->Bemf_alfa_est;
}

float SensorlessObserver_GetEstimatedBemfBeta(const SensorlessObserver_t *obs)
{
  return (obs == NULL) ? 0.0f : obs->Bemf_beta_est;
}

float SensorlessObserver_GetEstimatedCurrentAlpha(const SensorlessObserver_t *obs)
{
  return (obs == NULL) ? 0.0f : obs->Ialfa_est;
}

float SensorlessObserver_GetEstimatedCurrentBeta(const SensorlessObserver_t *obs)
{
  return (obs == NULL) ? 0.0f : obs->Ibeta_est;
}

float SensorlessObserver_GetEstimatedThetaRad(const SensorlessObserver_t *obs)
{
  return (obs == NULL) ? 0.0f : obs->ThetaEstRad;
}

float SensorlessObserver_GetEstimatedSpeedRadPerSec(const SensorlessObserver_t *obs)
{
  return (obs == NULL) ? 0.0f : obs->SpeedEstRadPerSec;
}

float SensorlessObserver_GetAverageSpeedRadPerSec(const SensorlessObserver_t *obs)
{
  return (obs == NULL) ? 0.0f : obs->SpeedAverage;
}
