#ifndef __APP_DEBUG_H__
#define __APP_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 调试任务接口。
 *
 * 这一层只负责低频读取快照并通过串口打印，
 * 不参与任何实时控制计算。
 *
 * 设计原则：
 * - 高频任务只负责写快照；
 * - 低频调试任务只负责读快照；
 * - 严禁在这里反向修改控制对象。
 */
void AppDebug_Task(void);

#ifdef __cplusplus
}
#endif

#endif
