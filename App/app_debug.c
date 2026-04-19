#include "app_debug.h"
#include "app_foc.h"
#include "foc_config.h"
#include "main.h"
#include "stm32g4xx_hal.h"
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
     * observer 对接验证打印（详细模式也统一输出这 5 个量）。
     *
     * 字段顺序：
     * - theta_open      : 开环角度；
     * - theta_obs       : 观测角度；
     * - angle_err_deg   : 开环角 - 观测角的误差（度）；
     * - open_speed_rad_s: 开环电角速度；
     * - obs_speed_rad_s : 观测器电角速度。
     */
    printf("%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
           rt.switchover.theta_open_rad,
           rt.switchover.theta_obs_rad,
           rt.switchover.angle_err_deg,
           rt.switchover.open_speed_rad_s,
           rt.switchover.obs_speed_rad_s);
#else
    /*
     * observer 对接验证打印（精简模式）。
     */
    printf("%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
           rt.switchover.theta_open_rad,
           rt.switchover.theta_obs_rad,
           rt.switchover.angle_err_deg,
           rt.switchover.open_speed_rad_s,
           rt.switchover.obs_speed_rad_s);
#endif
  }
}
