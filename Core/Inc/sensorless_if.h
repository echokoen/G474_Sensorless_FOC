#ifndef SENSORLESS_IF_H
#define SENSORLESS_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sensorless_observer.h"

void SensorlessIf_Init(void);
void SensorlessIf_Reset(void);
void SensorlessIf_Update(float valpha, float vbeta, float ialpha, float ibeta, float vbus);

float SensorlessIf_GetThetaEstRad(void);
float SensorlessIf_GetSpeedEstRadPerSec(void);
float SensorlessIf_GetAngleErrDeg(void);
uint8_t SensorlessIf_IsReliable(void);

void SensorlessIf_GetInputs(float *valpha, float *vbeta, float *ialpha, float *ibeta, float *vbus);
void SensorlessIf_GetFlux(float *flux_alpha, float *flux_beta);
void SensorlessIf_GetBEMF(float *bemf_alpha, float *bemf_beta);

#ifdef __cplusplus
}
#endif

#endif /* SENSORLESS_IF_H */