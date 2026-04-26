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

  /* Keep UART debug lightweight. */
  if ((HAL_GetTick() - last_dbg_tick) < 300u)
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

    {
      const float rad_s_to_rpm = 9.54929658551f;
      const float vd = rt.current_loop.vd_cmd_v;
      const float vq = rt.current_loop.vq_cmd_v;
      const float v_mag_sq = (vd * vd) + (vq * vq);
      const float v_lim = FOC_DQ_VOLT_LIMIT * 0.98f;
      const uint8_t voltage_limit_flag =
          (v_mag_sq >= (v_lim * v_lim)) ? 1u : 0u;

      printf("st=%d,ff=%u,sp=%.0f,sf=%.0f,iqr=%.2f,iq=%.2f,"
             "vqpi=%.2f,vqff=%.2f,vq=%.2f,lim=%u\r\n",
             (int)rt.state,
             (unsigned int)rt.current_loop.ff_enable,
             rt.speed_loop.speed_ref_mech_rad_s * rad_s_to_rpm,
             rt.speed_loop.speed_fdb_mech_rad_s * rad_s_to_rpm,
             rt.current_loop.iq_ref_a,
             rt.current_loop.iq_meas_a,
             rt.current_loop.vq_pi_v,
             rt.current_loop.vq_ff_v,
             rt.current_loop.vq_cmd_v,
             (unsigned int)voltage_limit_flag);
    }
  }
#else
  /* 调试打印关闭时保持接口存在，main.c 不需要条件编译。 */
#endif
}
