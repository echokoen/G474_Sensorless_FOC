#include "app_debug.h"
#include "app_foc.h"
#include "foc_config.h"
#include "main.h"
#include "foc_switchover.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdio.h>

static const char *app_debug_state_name(FOC_StateTypeDef state)
{
  switch (state)
  {
    case FOC_STATE_IDLE:
      return "IDLE";
    case FOC_STATE_OFFSET_CALIB:
      return "OFFSET_CALIB";
    case FOC_STATE_ALIGN:
      return "ALIGN";
    case FOC_STATE_OPENLOOP:
      return "OPENLOOP";
    case FOC_STATE_RUN:
      return "RUN";
    case FOC_STATE_FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

static const char *app_debug_switchover_state_name(uint8_t state)
{
  switch ((FOC_SwitchoverState_t)state)
  {
    case FOC_SW_STATE_OPENLOOP:
      return "OPENLOOP";
    case FOC_SW_STATE_BLEND:
      return "BLEND";
    case FOC_SW_STATE_OBSERVER:
      return "OBSERVER";
    default:
      return "UNKNOWN";
  }
}

/*
 * 调试任务实现。
 *
 * 作用：
 * 1. 低频读取高频缓存好的调试快照；
 * 2. 整理成适合串口观察的文本；
 * 3. 尽量避免给高频控制链增加额外负担。
 *
 * 注意：
 * 这里的打印周期故意做了限速，
 * 否则串口打印本身会明显拖慢系统。
 */
void AppDebug_Task(void)
{
  static uint32_t last_dbg_tick = 0u;

  /* 约 5 ms 打印一次，避免串口输出过于频繁。 */
  if ((HAL_GetTick() - last_dbg_tick) < 5u)
  {
    return;
  }

  last_dbg_tick = HAL_GetTick();
  {
    FOC_DebugSnapshot_t dbg = {0};
    FOC_RuntimeSnapshot_t rt = {0};

    /*
     * dbg：更偏底层采样/PWM 细节。
     * rt ：更偏应用层运行状态汇总。
     */
    (void)AppFoc_GetDebugSnapshot(&dbg);
    (void)AppFoc_GetRuntimeSnapshot(&rt);

#if (FOC_DEBUG_PRINT_DETAIL != 0u)
    /*
     * RUN 接管诊断打印（简化模式）。
     *
     * 字段顺序：
     * - speed_ref : 当前机械速度给定，单位 rad/s；
     * - speed_fdb : observer 输出换算后的机械速度反馈，单位 rad/s；
     * - id_ref    : 当前 d 轴电流给定；
     * - iq_ref    : 当前 q 轴电流给定；
     * - id_meas   : 当前 d 轴电流实测；
     * - iq_meas   : 当前 q 轴电流实测。
     */
    printf("state=%s(%u),sw_state=%s(%u),obs_ready=%u,ready_now=%u,hold=%lu,blend_ticks=%lu,blend_k=%.3f,ang_err_deg=%.3f,theta_open=%.3f,theta_obs=%.3f,theta_ctrl=%.3f,open_spd=%.3f,obs_spd=%.3f,spd_err=%.3f\r\n",
           app_debug_state_name(rt.state),
           (unsigned int)rt.state,
           app_debug_switchover_state_name(rt.switchover.state),
           (unsigned int)rt.switchover.state,
           (unsigned int)rt.switchover.observer_ready,
           (unsigned int)rt.switchover.ready_now,
           (unsigned long)rt.switchover.hold_ticks,
           (unsigned long)rt.switchover.blend_ticks,
           rt.switchover.blend_k,
           rt.switchover.angle_err_deg,
           rt.switchover.theta_open_rad,
           rt.switchover.theta_obs_rad,
           rt.switchover.theta_ctrl_rad,
           rt.switchover.open_speed_rad_s,
           rt.switchover.obs_speed_rad_s,
           rt.switchover.speed_err_rad_s);
#else
    printf("state=%s(%u),sw_state=%s(%u),obs_ready=%u,ready_now=%u,hold=%lu,blend_ticks=%lu,blend_k=%.3f,ang_err_deg=%.3f,theta_open=%.3f,theta_obs=%.3f,theta_ctrl=%.3f,open_spd=%.3f,obs_spd=%.3f,spd_err=%.3f\r\n",
           app_debug_state_name(rt.state),
           (unsigned int)rt.state,
           app_debug_switchover_state_name(rt.switchover.state),
           (unsigned int)rt.switchover.state,
           (unsigned int)rt.switchover.observer_ready,
           (unsigned int)rt.switchover.ready_now,
           (unsigned long)rt.switchover.hold_ticks,
           (unsigned long)rt.switchover.blend_ticks,
           rt.switchover.blend_k,
           rt.switchover.angle_err_deg,
           rt.switchover.theta_open_rad,
           rt.switchover.theta_obs_rad,
           rt.switchover.theta_ctrl_rad,
           rt.switchover.open_speed_rad_s,
           rt.switchover.obs_speed_rad_s,
           rt.switchover.speed_err_rad_s);
#endif
  }
}
