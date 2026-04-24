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
  volatile FOC_DebugSnapshot_t snapshot; /* 高频写、低频读的共享快照。 */
} FOC_DebugData_t;

/* 初始化调试快照缓存。 */
void FOC_DebugModule_Init(FOC_DebugData_t *debug_data);

/* 高频侧更新快照。
 * 注意：该函数应保持轻量，只复制已有运行量，不做复杂计算。
 */
void FOC_DebugModule_UpdateSnapshot(FOC_DebugData_t *debug_data,
                                    const FOC_SamplingData_t *sampling,
                                    const FOC_PwmData_t *pwm);

/* 低频侧读取快照。
 * 返回 1 表示读到了稳定的一拍数据。
 */
uint8_t FOC_DebugModule_GetSnapshot(const FOC_DebugData_t *debug_data, FOC_DebugSnapshot_t *snapshot);
#ifdef __cplusplus
}

#endif
#endif
