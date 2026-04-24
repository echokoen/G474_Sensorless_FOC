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

  /* PWM 诊断打印降速到约 20 ms，避免串口拖慢系统。 */
  if ((HAL_GetTick() - last_dbg_tick) < 20u)
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
    printf("state=%s(%u),sw_state=%s(%u),sample_sector=%u,pwm_sector=%u,pair=%u,ccr_u=%u,ccr_v=%u,ccr_w=%u,smp=%u,win1=%u,win2=%u,t1=%.1f,t2=%.1f,t0=%.1f,raw_u=%u,raw_v=%u,raw_w=%u\r\n",
           app_debug_state_name(rt.state),
           (unsigned int)rt.state,
           app_debug_switchover_state_name(rt.switchover.state),
           (unsigned int)rt.switchover.state,
           (unsigned int)dbg.sec,
           (unsigned int)dbg.pwm_sec,
           (unsigned int)dbg.pair,
           (unsigned int)dbg.ccru,
           (unsigned int)dbg.ccrv,
           (unsigned int)dbg.ccrw,
           (unsigned int)dbg.smp,
           (unsigned int)dbg.win1,
           (unsigned int)dbg.win2,
           dbg.t1,
           dbg.t2,
           dbg.t0,
           (unsigned int)dbg.rawu,
           (unsigned int)dbg.rawv,
           (unsigned int)dbg.raww);
#else
    printf("state=%s(%u),sw_state=%s(%u),sample_sector=%u,pwm_sector=%u,pair=%u,ccr_u=%u,ccr_v=%u,ccr_w=%u,smp=%u,win1=%u,win2=%u,t1=%.1f,t2=%.1f,t0=%.1f,raw_u=%u,raw_v=%u,raw_w=%u\r\n",
           app_debug_state_name(rt.state),
           (unsigned int)rt.state,
           app_debug_switchover_state_name(rt.switchover.state),
           (unsigned int)rt.switchover.state,
           (unsigned int)dbg.sec,
           (unsigned int)dbg.pwm_sec,
           (unsigned int)dbg.pair,
           (unsigned int)dbg.ccru,
           (unsigned int)dbg.ccrv,
           (unsigned int)dbg.ccrw,
           (unsigned int)dbg.smp,
           (unsigned int)dbg.win1,
           (unsigned int)dbg.win2,
           dbg.t1,
           dbg.t2,
           dbg.t0,
           (unsigned int)dbg.rawu,
           (unsigned int)dbg.rawv,
           (unsigned int)dbg.raww);
#endif
  }
}
