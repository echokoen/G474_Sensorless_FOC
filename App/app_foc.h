#ifndef __APP_FOC_H__
#define __APP_FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_types.h"

/*
 * FOC 应用层主入口。
 *
 * 这一层不直接实现采样、电流环、观测器等算法，
 * 而是负责把各个控制模块按固定顺序组织起来，
 * 对外只暴露少量必要接口，避免外部代码直接操作内部对象。
 *
 * 设计目标：
 * 1. 对外统一主控入口；
 * 2. 高频任务执行路径集中；
 * 3. 调试信息统一从快照读取，避免大量零散 getter。
 *
 * 使用方式：
 * - main 初始化外设后调用 AppFoc_Init()；
 * - 用户按键或命令触发 AppFoc_Start()；
 * - ADC 注入完成中断里调用 AppFoc_HighFreqTask()；
 * - 主循环或 1ms tick 中调用 AppFoc_MediumFreqTask()；
 * - AppDebug_Task() 通过快照接口读取状态，不直接碰内部对象。
 */

/* 初始化应用层控制对象。
 *
 * 调用时机：系统上电完成外设初始化后。
 * 主要动作：
 * - 初始化采样、PWM、调试、电流环、开环、观测器、切换模块；
 * - 将 PWM 先置于中点输出；
 * - 关闭驱动使能；
 * - 状态机回到 IDLE。
 */
void AppFoc_Init(void);

/* 启动 FOC 控制流程。
 *
 * 调用后会启动：
 * - TIM1 三相 PWM；
 * - TIM1 基本定时中断；
 * - ADC 常规/注入采样；
 * - 相关内部状态复位。
 *
 * 注意：启动后不会立刻进入闭环，
 * 而是先进入 OFFSET_CALIB，再进入 ALIGN / OPENLOOP。
 */
void AppFoc_Start(void);

/* 停止 FOC 控制流程。
 *
 * 该函数会关闭 PWM、停止 ADC、撤销驱动输出，
 * 并将状态机拉回 IDLE。
 * 一般用于主动停机或故障停机后的收口处理。
 */
void AppFoc_Stop(void);

/* 高频控制任务。
 *
 * 典型调用位置：ADC 注入转换完成中断。
 *
 * 这里是整个无感 FOC 的高频主链入口，
 * 会根据当前状态机执行：
 * - 零点校准；
 * - 对齐；
 * - 开环运行；
 * - 故障保持；
 * 同时在每次任务结束后刷新调试快照。
 */
void AppFoc_HighFreqTask(void);

/* 中频控制任务。
 *
 * 典型调用位置：main while(1) 中 1 ms 周期轮询。
 *
 * 职责：
 * - 处理主状态机推进；
 * - 处理启动/停机/故障收口这类慢流程；
 * - 预留速度环与给定更新框架；
 * - 高频只负责执行本拍控制。
 */
void AppFoc_MediumFreqTask(void);

/* 获取当前 FOC 运行状态。 */
FOC_StateTypeDef AppFoc_GetState(void);

/* 获取高频调试快照。
 *
 * 返回值：
 * - 1：获取成功；
 * - 0：参数为空或未获取到有效数据。
 */
uint8_t AppFoc_GetDebugSnapshot(FOC_DebugSnapshot_t *snapshot);

/* 获取应用层运行快照。
 *
 * 与调试快照不同，这个接口更偏向“运行状态汇总”，
 * 把开环、观测器、采样、电流环、切换模块的关键量
 * 一次性打包出来，供低频打印或上位机读取。
 */
uint8_t AppFoc_GetRuntimeSnapshot(FOC_RuntimeSnapshot_t *snapshot);

/* 设置 RUN 阶段的机械速度目标，单位 Hz。
 *
 * 当前版本默认在进入 RUN 时先用当前 observer 速度接管，
 * 后续如果需要变速，可通过该接口修改速度环目标。
 */
void AppFoc_SetSpeedTargetMechHz(float mech_hz);

/* 获取当前锁存的故障原因位图。 */
uint32_t AppFoc_GetFaultFlags(void);

#ifdef __cplusplus
}
#endif

#endif
