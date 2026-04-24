#ifndef __FOC_SWITCHOVER_H__
#define __FOC_SWITCHOVER_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "foc_observer.h"
#include "foc_openloop.h"

/* 观测器接管模块接口。
 *
 * 用于管理开环角与观测器角之间的切换状态。
 */

typedef enum
{
  FOC_SW_STATE_OPENLOOP = 0, /* 仍使用开环角作为控制角。 */
  FOC_SW_STATE_BLEND,        /* 开环角和 observer 角按 blend_k 平滑混合。 */
  FOC_SW_STATE_OBSERVER      /* observer 角完全接管控制角。 */
} FOC_SwitchoverState_t;

typedef struct
{
  uint8_t observer_ready;   /* 最终接管就绪标志：满足保持时间后才会置位。 */
  uint8_t ready_now;        /* 当前这一拍是否满足接管条件，主要用于调试观察。 */
  uint32_t hold_ticks;      /* 连续满足条件的保持计数。 */
  uint32_t blend_ticks;     /* 混合接管已经持续的 tick 数。 */
  float blend_k;            /* 开环角 -> observer 角的混合系数。 */
  float open_speed_rad_s;   /* 当前开环电角速度，供调试查看。 */
  float obs_speed_rad_s;    /* 当前 observer 电角速度，供调试查看。 */
  float speed_err_rad_s;    /* 当前速度误差绝对值，供调试查看。 */
  uint8_t fail_reason;      /* 本拍 ready 失败原因位图：bit0=频率，bit1=角度，bit2=速度。 */
  uint8_t last_blend_reset_reason; /* 最近一次 BLEND 被打回 OPENLOOP 的原因位图。 */
  uint32_t blend_fail_ticks;       /* BLEND 中连续失败计数，用于抑制单拍毛刺。 */
  uint32_t blend_reset_count;      /* BLEND 被打回 OPENLOOP 的累计次数。 */
  float last_blend_reset_angle_err_deg; /* 最近一次 BLEND 回退瞬间的角度误差。 */
  FOC_SwitchoverState_t state; /* 当前接管状态机状态。 */
} FOC_SwitchoverData_t;

/* 初始化/复位接管状态。 */
void FOC_SwitchoverInit(FOC_SwitchoverData_t *sw);
void FOC_SwitchoverReset(FOC_SwitchoverData_t *sw);

/* 更新接管状态机：判断是否 ready、是否进入 BLEND、是否完成接管。 */
void FOC_SwitchoverUpdate(FOC_SwitchoverData_t *sw,
                          const FOC_OpenLoopData_t *openloop,
                          const FOC_ObserverData_t *observer);

/* 根据当前接管状态选择最终控制角。 */
float FOC_SwitchoverSelectTheta(const FOC_SwitchoverData_t *sw,
                                const FOC_OpenLoopData_t *openloop,
                                const FOC_ObserverData_t *observer);

/* 打包接管调试量，供 app_foc 生成运行快照。 */
void FOC_SwitchoverGetDebug(const FOC_SwitchoverData_t *sw,
                            const FOC_OpenLoopData_t *openloop,
                            const FOC_ObserverData_t *observer,
                            float theta_ctrl,
                            uint8_t *sw_state,
                            uint8_t *observer_ready,
                            float *blend_k,
                            float *theta_open,
                            float *theta_obs,
                            float *theta_ctrl_out,
                            float *angle_err_deg,
                            float *open_speed_rad_s,
                            float *obs_speed_rad_s);
#ifdef __cplusplus
}

#endif
#endif
