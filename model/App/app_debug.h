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
 * 默认由 APP_DEBUG_PRINT_ENABLE 关闭，需要串口日志时再打开。
 *
 * 设计原则：
 * - 高频任务只负责写快照；
 * - 低频调试任务只负责读快照；
 * - 严禁在这里反向修改控制对象。
 *
 * 如果要新增打印字段，优先从 AppFoc_GetRuntimeSnapshot()
 * 或 AppFoc_GetDebugSnapshot() 中取，避免直接访问控制模块内部变量。
 */
void AppDebug_Task(void);

#ifdef __cplusplus
}
#endif

#endif
