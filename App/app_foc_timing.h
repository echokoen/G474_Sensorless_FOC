#ifndef __APP_FOC_TIMING_H__
#define __APP_FOC_TIMING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_types.h"
#include <stdint.h>

void AppFocTiming_Init(void);
uint32_t AppFocTiming_GetCycle(void);
void AppFocTiming_Update(uint32_t cycles);
uint8_t AppFocTiming_GetSnapshot(FOC_HfTimingSnapshot_t *snapshot);
void AppFocTiming_Reset(void);

#ifdef __cplusplus
}
#endif

#endif
