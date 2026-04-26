#include "app_debug.h"
#include "app_foc.h"
#include "foc_config.h"
#include "stm32g4xx_hal.h"

#if (APP_DEBUG_PRINT_ENABLE != 0u)
#include <stdio.h>
#include <string.h>
#endif

/*
 * 低频调试打印任务。
 *
 * 默认关闭，避免阻塞式串口打印影响中频任务和调参判断。
 * 需要串口观察时，把 foc_config.h 中 APP_DEBUG_PRINT_ENABLE 置 1。
 */
void AppDebug_Task(void)
{
#if (APP_DEBUG_PRINT_ENABLE != 0u)
  static uint32_t last_dbg_tick = 0u;

  /* 打印周期约 100 ms。 */
  if ((HAL_GetTick() - last_dbg_tick) < 100u)
  {
    return;
  }

  last_dbg_tick = HAL_GetTick();

  {
    FOC_RuntimeSnapshot_t rt;
    memset(&rt, 0, sizeof(rt));

    if (AppFoc_GetRuntimeSnapshot(&rt) == 0u)
    {
      return;
    }

    printf("state=%u,fault=0x%08lX,sw=%u,ready=%u,hold=%lu,fail=0x%02X,"
           "angle_err=%.2f,open_w=%.1f,obs_w=%.1f,speed_err=%.1f,blend=%.3f,"
           "speed_en=%u,ref=%.1f,target=%.1f,fdb=%.1f,iq_ref=%.2f,"
           "vbus=%.1f,iu=%.2f,iv=%.2f,iw=%.2f,hold_last=%u,hold_cnt=%u\r\n",
           (unsigned int)rt.state,
           (unsigned long)rt.fault_flags,
           (unsigned int)rt.switchover.state,
           (unsigned int)rt.switchover.ready_now,
           (unsigned long)rt.switchover.hold_ticks,
           (unsigned int)rt.switchover.fail_reason,
           rt.switchover.angle_err_deg,
           rt.switchover.open_speed_rad_s,
           rt.switchover.obs_speed_rad_s,
           rt.switchover.speed_err_rad_s,
           rt.switchover.blend_k,
           (unsigned int)rt.speed_loop.enabled,
           rt.speed_loop.speed_ref_mech_rad_s,
           rt.speed_loop.speed_target_mech_rad_s,
           rt.speed_loop.speed_fdb_mech_rad_s,
           rt.speed_loop.iq_ref_cmd_a,
           rt.sampling.vbus_v,
           rt.sampling.iu_a,
           rt.sampling.iv_a,
           rt.sampling.iw_a,
           (unsigned int)rt.sampling.used_hold_last_current,
           (unsigned int)rt.sampling.fallback_hold_count);

#if ((APP_DWT_TIMING_ENABLE != 0u) && (APP_DWT_TIMING_PRINT_ENABLE != 0u))
    {
      FOC_HfTimingSnapshot_t hf;
      const float cycles_to_us = 1000000.0f / (float)SystemCoreClock;

      memset(&hf, 0, sizeof(hf));
      if (AppFoc_GetHfTimingSnapshot(&hf) != 0u)
      {
        printf("hf_cycle last=%lu,min=%lu,max=%lu,avg=%lu,thr=%lu,ov=%lu,"
               "hf_us last=%.2f,max=%.2f,avg=%.2f\r\n",
               (unsigned long)hf.last_cycles,
               (unsigned long)hf.min_cycles,
               (unsigned long)hf.max_cycles,
               (unsigned long)hf.avg_cycles,
               (unsigned long)hf.threshold_cycles,
               (unsigned long)hf.overrun_count,
               (float)hf.last_cycles * cycles_to_us,
               (float)hf.max_cycles * cycles_to_us,
               (float)hf.avg_cycles * cycles_to_us);
      }
    }
#endif
  }
#else
  /* 调试打印关闭时保持接口存在，main.c 不需要条件编译。 */
#endif
}
