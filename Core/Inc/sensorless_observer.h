#ifndef SENSORLESS_OBSERVER_H
#define SENSORLESS_OBSERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define SENSORLESS_SPEED_BUFFER_SIZE  (16u)

typedef struct
{
  float Vbus;
  float Valpha;
  float Vbeta;
  float Ialpha;
  float Ibeta;

  float Ialfa_est;
  float Ibeta_est;
  float Bemf_alfa_est;
  float Bemf_beta_est;
  float FluxAlpha;
  float FluxBeta;
  float ThetaEstRad;
  float SpeedEstRadPerSec;
  float AngleErrRad;

  float SpeedBuffer[SENSORLESS_SPEED_BUFFER_SIZE];
  uint8_t SpeedBufferIndex;
  uint8_t SpeedBufferSize;
  float SpeedBufferSum;
  float SpeedAverage;
  float SpeedBufferOldestEl;
  uint8_t SpeedBufferSizeDpp;

  uint8_t IsSpeedReliable;
  uint8_t IsBemfConsistent;
  uint8_t IsConverged;
  uint8_t ReliabilityCounter;
  uint8_t ConsistencyCounter;
  uint8_t EnableDualCheck;
  int8_t ForcedDirection;

  float PllIntegral;
  float FluxMagSq;
  float BemfMagSq;
  int16_t hElSpeedDpp;
  int16_t hElAngle;
  uint8_t ForceConvergency;
  uint8_t ForceConvergency2;
} SensorlessObserver_t;

void SensorlessObserver_Init(SensorlessObserver_t *obs);
void SensorlessObserver_Reset(SensorlessObserver_t *obs);
void SensorlessObserver_Update(SensorlessObserver_t *obs);

bool SensorlessObserver_IsConverged(const SensorlessObserver_t *obs);
float SensorlessObserver_GetEstimatedBemfAlpha(const SensorlessObserver_t *obs);
float SensorlessObserver_GetEstimatedBemfBeta(const SensorlessObserver_t *obs);
float SensorlessObserver_GetEstimatedCurrentAlpha(const SensorlessObserver_t *obs);
float SensorlessObserver_GetEstimatedCurrentBeta(const SensorlessObserver_t *obs);
float SensorlessObserver_GetEstimatedThetaRad(const SensorlessObserver_t *obs);
float SensorlessObserver_GetEstimatedSpeedRadPerSec(const SensorlessObserver_t *obs);
float SensorlessObserver_GetAverageSpeedRadPerSec(const SensorlessObserver_t *obs);

#ifdef __cplusplus
}
#endif

#endif /* SENSORLESS_OBSERVER_H */