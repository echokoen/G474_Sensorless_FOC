#ifndef FOC_TRACE_H
#define FOC_TRACE_H
#ifdef __cplusplus
extern "C" {
#endif
#include "foc_types.h"
#include "foc_svpwm.h"
#include "foc_current_sense.h"

/* 调试快照模块接口。
 *
 * 该模块负责维护高频侧写入、低频侧读取的调试快照。
 */

typedef struct
{
  volatile FOC_TraceSnapshot_t snapshot; /* 高频写、低频读的共享快照。 */
} FOC_Trace_t;

/* 初始化调试快照缓存。 */
void FOC_Trace_Init(FOC_Trace_t *trace);

/* 高频侧更新快照。
 * 注意：该函数应保持轻量，只复制已有运行量，不做复杂计算。
 */
void FOC_Trace_UpdateSnapshot(FOC_Trace_t *trace,
                                    const FOC_CurrentSense_t *current_sense,
                                    const FOC_Svpwm_t *svpwm);

/* 低频侧读取快照。
 * 返回 1 表示读到了稳定的一拍数据。
 */
uint8_t FOC_Trace_GetSnapshot(const FOC_Trace_t *trace, FOC_TraceSnapshot_t *snapshot);
#ifdef __cplusplus
}

#endif
#endif /* FOC_TRACE_H */
