#include "app_debug.h"
#include "app_foc.h"
#include "foc_config.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdio.h>

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
     * RUN 接管诊断打印（详细模式）。
     *
     * 字段顺序：
     * - foc_state  : 主状态机；
     * - sw         : switchover 状态，0=openloop，1=blend，2=observer；
     * - speed_ref  : 当前速度环机械速度给定，单位 rad/s；
     * - speed_fdb  : observer 输出换算后的机械速度反馈，单位 rad/s；
     * - iq_ref     : 当前 q 轴电流给定；
     * - iq_meas    : 当前 q 轴电流实测。
     * - theta_open : 当前开环电角；
     * - theta_obs  : 当前 observer 电角。
     */
    printf("foc_state=%u,sw=%u,speed_ref=%.3f,speed_fdb=%.3f,iq_ref=%.3f,iq_meas=%.3f,theta_open=%.3f,theta_obs=%.3f\r\n",
           (unsigned int)AppFoc_GetState(),
           (unsigned int)rt.switchover.state,
           rt.speed_loop.speed_ref_mech_rad_s,
           rt.speed_loop.speed_fdb_mech_rad_s,
           rt.current_loop.iq_ref_a,
           rt.current_loop.iq_meas_a,
           rt.switchover.theta_open_rad,
           rt.switchover.theta_obs_rad);
#else
    printf("foc_state=%u,sw=%u,speed_ref=%.3f,speed_fdb=%.3f,iq_ref=%.3f,iq_meas=%.3f,theta_open=%.3f,theta_obs=%.3f\r\n",
           (unsigned int)AppFoc_GetState(),
           (unsigned int)rt.switchover.state,
           rt.speed_loop.speed_ref_mech_rad_s,
           rt.speed_loop.speed_fdb_mech_rad_s,
           rt.current_loop.iq_ref_a,
           rt.current_loop.iq_meas_a,
           rt.switchover.theta_open_rad,
           rt.switchover.theta_obs_rad);
#endif
  }
}
