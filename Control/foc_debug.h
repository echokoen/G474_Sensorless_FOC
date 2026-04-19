#ifndef __FOC_DEBUG_H__
#define __FOC_DEBUG_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "foc_types.h"
#include "foc_pwm.h"
#include "foc_sampling.h"

/* 调试快照模块接口。
 *
 * 该模块负责维护高频侧写入、低频侧读取的调试快照。
 */

typedef struct
{
  volatile FOC_DebugSnapshot_t snapshot;
} FOC_DebugData_t;
void FOC_DebugModule_Init(FOC_DebugData_t *debug_data);

void FOC_DebugModule_UpdateSnapshot(FOC_DebugData_t *debug_data,
                                    const FOC_SamplingData_t *sampling,
                                    const FOC_PwmData_t *pwm);
uint8_t FOC_DebugModule_GetSnapshot(const FOC_DebugData_t *debug_data, FOC_DebugSnapshot_t *snapshot);
#ifdef __cplusplus
}

#endif
#endif
