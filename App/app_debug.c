#include "app_debug.h"
#include "app_foc.h"
#include "foc_config.h"
#include "main.h"
#include "foc_switchover.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

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

  /* 诊断打印降速到约 100 ms，避免阻塞式串口拖慢中频速度环。 */
  if ((HAL_GetTick() - last_dbg_tick) < 100u)
  {
    return;
  }

  last_dbg_tick = HAL_GetTick();
  {
    FOC_DebugSnapshot_t dbg;
    FOC_RuntimeSnapshot_t rt;
    memset(&dbg, 0, sizeof(dbg));
    memset(&rt, 0, sizeof(rt));

    /*
     * dbg：更偏底层采样/PWM 细节。
     * rt ：更偏应用层运行状态汇总。
     */
    (void)AppFoc_GetDebugSnapshot(&dbg);
    (void)AppFoc_GetRuntimeSnapshot(&rt);

    {
      /*
       * switchover 专用诊断：
       * 只看接管状态、当前条件、失败原因、角度/速度误差和 blend 进度。
       */
      printf("state=%u,sw=%u,ready_now=%u,hold_ticks=%lu,fail_reason=0x%02X,"
              "angle_err_deg=%.2f,open_speed_rad_s=%.1f,obs_speed_rad_s=%.1f,"
              "speed_err_rad_s=%.1f,blend_k=%.3f,blend_reset_count=%lu,"
              "speed_en=%u,speed_ref_mech_rad_s=%.1f,speed_target_mech_rad_s=%.1f,"
              "speed_fdb_mech_rad_s=%.1f,iq_ref_cmd_a=%.2f\r\n",
              (unsigned int)rt.state,
              (unsigned int)rt.switchover.state,
              (unsigned int)rt.switchover.ready_now,
              (unsigned long)rt.switchover.hold_ticks,
              (unsigned int)rt.switchover.fail_reason,
              rt.switchover.angle_err_deg,
             rt.switchover.open_speed_rad_s,
              rt.switchover.obs_speed_rad_s,
              rt.switchover.speed_err_rad_s,
              rt.switchover.blend_k,
              (unsigned long)rt.switchover.blend_reset_count,
              (unsigned int)rt.speed_loop.enabled,
              rt.speed_loop.speed_ref_mech_rad_s,
              rt.speed_loop.speed_target_mech_rad_s,
              rt.speed_loop.speed_fdb_mech_rad_s,
              rt.speed_loop.iq_ref_cmd_a);
    }

  }
}
