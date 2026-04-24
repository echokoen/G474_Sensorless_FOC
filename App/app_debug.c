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

  /* PWM 诊断打印降速到约 20 ms，避免串口拖慢系统。 */
  if ((HAL_GetTick() - last_dbg_tick) < 20u)
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
      const float obs_spd_elec_rad_s = rt.observer.speed_rad_s;
      const float obs_spd_mech_rad_s = obs_spd_elec_rad_s / FOC_POLE_PAIRS;
      const float vdq_mag_v = sqrtf((rt.current_loop.vd_cmd_v * rt.current_loop.vd_cmd_v) +
                                    (rt.current_loop.vq_cmd_v * rt.current_loop.vq_cmd_v));
      /*
       * RUN 速度/电压诊断：
       * - speed_ref / obs_mech / speed_err：机械速度闭环是否追上；
       * - iq_ref / iq：速度环要的电流和电流环实际跟踪；
       * - vd/vq/vdq/vbus：判断是否电压余量不足。
       */
      printf("st=%u,sw=%u,obs_e=%.1f,obs_m=%.1f,pll_i=%.1f,pll_err=%.3f,"
             "lim_s=%u,lim_i=%u,iq_ref=%.2f,iq=%.2f,id_ref=%.2f,id=%.2f,"
             "vd=%.2f,vq=%.2f,vdq=%.2f/%.1f,vbus=%.1f\r\n",
             (unsigned int)rt.state,
             (unsigned int)rt.switchover.state,
             obs_spd_elec_rad_s,
             obs_spd_mech_rad_s,
             rt.observer.pll_integral,
             rt.observer.pll_err,
             (unsigned int)rt.observer.speed_limit_hit,
             (unsigned int)rt.observer.integral_limit_hit,
             rt.current_loop.iq_ref_a,
             rt.current_loop.iq_meas_a,
             rt.current_loop.id_ref_a,
             rt.current_loop.id_meas_a,
             rt.current_loop.vd_cmd_v,
             rt.current_loop.vq_cmd_v,
             vdq_mag_v,
             FOC_DQ_VOLT_LIMIT,
             rt.sampling.vbus_v);
    }

  }
}
