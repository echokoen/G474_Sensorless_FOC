#ifndef __FOC_BRIDGE_H__
#define __FOC_BRIDGE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bsp_adc.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"
#include "foc_config.h"

/* FOC 主状态机。
 * 设计思路：
 * - `IDLE`：未启动，所有输出保持中点。
 * - `OFFSET_CALIB`：采样电流传感器零点偏置，避免后续电流计算带固定偏差。
 * - `ALIGN`：上电定子磁场对齐，让转子先落到一个已知位置。
 * - `OPENLOOP`：按开环角度和频率旋转，给 observer 提供可跟踪轨迹。
 * - `FAULT`：过压、欠压或过流后锁定输出，等待人工处理。
 */
typedef enum
{
  FOC_STATE_IDLE = 0,
  FOC_STATE_OFFSET_CALIB,
  FOC_STATE_ALIGN,
  FOC_STATE_OPENLOOP,
  FOC_STATE_FAULT
} FOC_StateTypeDef;

/* 调试快照。
 *
 * 这一份结构体表示“同一个 ADC / PWM 周期”里锁存下来的调试数据，
 * 主循环只能打印这份快照，不能再分别读取散落的全局变量。
 */
typedef struct
{
  uint32_t seq;           /* 快照序号。使用类似 seqlock 的方式更新，稳定快照通常为偶数序号。 */
  uint8_t sec;            /* 当前 SVPWM 扇区。 */
  uint8_t pair;           /* 当前扇区认定的可信两相组合。1=VW, 2=UW, 3=UV。 */
  uint8_t fb;             /* 当前采样点是否走了 fallback。0=正常窗口，1=回退点。 */
  uint16_t ccru;          /* U 相 PWM 比较值。 */
  uint16_t ccrv;          /* V 相 PWM 比较值。 */
  uint16_t ccrw;          /* W 相 PWM 比较值。 */
  uint16_t win1;          /* 排序后第一个主采样窗口宽度。 */
  uint16_t win2;          /* 排序后第二个主采样窗口宽度。 */
  uint16_t smp;           /* 当前锁存的 ADC 触发点 CCR4。 */
  uint16_t rawu;          /* U 相原始 ADC 计数。 */
  uint16_t rawv;          /* V 相原始 ADC 计数。 */
  uint16_t raww;          /* W 相原始 ADC 计数。 */
  float iu;               /* U 相电流，单位安。 */
  float iv;               /* V 相电流，单位安。 */
  float iw;               /* W 相电流，单位安。 */
  float isum;             /* 三相电流和，理论上应接近 0。 */
} FOC_DebugSnapshot_t;

/* 生命周期控制。
 *
 * 这一组接口把“软件状态”和“硬件动作”分开：
 * - `FOC_BridgeInit` 只做软件侧的清零和默认值设置；
 * - `FOC_Start` 才真正使能 PWM / ADC / 中断；
 * - `FOC_Stop` 则把功率级和采样链路收回到安全状态。
 *
 * 这样做的好处是，调试时能先确认内部状态是否正确，再去看硬件是否真的被驱动。
 */
void FOC_BridgeInit(void);
void FOC_Start(void);
void FOC_Stop(void);
void FOC_TaskHighFreq(void);
void FOC_TaskBackground(void);
FOC_StateTypeDef FOC_GetState(void);

/* 纯采集接口。
 *
 * 这一组接口只负责把电压 / 电流采样结果整理出来，
 * 不参与 observer，也不参与开环控制。
 * 如果你只是想移植正点原子的采样链路，优先调用这一组。
 */
void FOC_SamplingUpdate(void);
float FOC_GetBusVoltageV(void);
float FOC_GetPhaseCurrentUa(void);
float FOC_GetPhaseCurrentVa(void);
float FOC_GetPhaseCurrentWa(void);
void FOC_GetCurrentOffsets(float *iu_offset_v, float *iv_offset_v);
void FOC_GetCurrentOffsets3(float *iu_offset_v, float *iv_offset_v, float *iw_offset_v);
void FOC_GetCurrentRawAdc(uint16_t *raw_u, uint16_t *raw_v, uint16_t *raw_w);
void FOC_GetInjectedRawAdcDebug(uint16_t *adc1_rank1,
                                uint16_t *adc1_rank2,
                                uint16_t *adc2_rank1,
                                uint16_t *adc2_rank2,
                                uint16_t *adc2_rank3);
void FOC_GetSvpwmDebug(uint8_t *sector, float *duty_u, float *duty_v, float *duty_w);
void FOC_GetSamplingWindowDebug(uint8_t *sector,
                                uint16_t *ccr_u,
                                uint16_t *ccr_v,
                                uint16_t *ccr_w,
                                uint16_t *win1,
                                uint16_t *win2,
                                uint16_t *sample_ccr,
                                uint8_t *used_fallback,
                                uint8_t *trusted_pair);
uint8_t FOC_GetDebugSnapshot(FOC_DebugSnapshot_t *snapshot);

/* 第一阶段核心步骤拆分。
 *
 * 这些函数通常都在高频采样中断里被调用，所以它们的共同原则是：
 * - 不做阻塞等待；
 * - 不做复杂动态分配；
 * - 只完成当前状态的一小步推进。
 */
void FOC_CurrentOffsetCalib(void);
void FOC_AlignRun(void);
void FOC_OpenLoopRun(void);
void FOC_FaultStop(void);

/* 第二阶段 observer 调试接口。
 *
 * 这些 getter 不直接计算结果，只返回最近一次更新后的缓存值。
 * 这么做可以避免串口打印和上位机采样把高频控制路径拖慢。
 */
float FOC_GetOpenLoopFreqHz(void);
float FOC_GetOpenLoopThetaErad(void);
float FOC_GetFluxThetaRad(void);
float FOC_GetObservedThetaRad(void);
float FOC_GetObservedSpeedRadPerSec(void);
float FOC_GetObservedAngleErrDeg(void);
uint8_t FOC_IsObserverLocked(void);
void FOC_GetObserverInputs(float *valpha, float *vbeta, float *ialpha, float *ibeta, float *vbus);
void FOC_GetCurrentLoopDebug(float *id_ref,
                             float *iq_ref,
                             float *id_meas,
                             float *iq_meas,
                             float *vd_cmd,
                             float *vq_cmd,
                             float *valpha_cmd,
                             float *vbeta_cmd,
                             float *theta_ctrl);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_BRIDGE_H__ */
