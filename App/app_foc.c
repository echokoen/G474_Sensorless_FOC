#include "app_foc.h"
#include "app_foc_timing.h"

/*
 * FOC 应用层编排实现。
 *
 * 说明：
 * - 这里不重复实现底层算法；
 * - 这里的职责是“组织流程”，即决定每一拍高频任务里，
 *   先做什么、后做什么、在什么状态下执行哪条链路；
 * - 采样、电流环、开环、观测器、切换逻辑都作为独立模块存在，
 *   应用层只负责调度它们。
 */

#include "foc_flux_observer_pll.h"
#include "bsp_key.h"
#include "bsp_adc.h"
#include "bsp_gpio.h"
#include "bsp_tim.h"
#include "foc_current_loop.h"
#include "foc_config.h"
#if ((FOC_DT_COMP_ENABLE != 0u) && ((FOC_DT_COMP_ENABLE_ALIGN != 0u) || (FOC_DT_COMP_ENABLE_OPENLOOP != 0u) || (FOC_DT_COMP_ENABLE_RUN != 0u)))
#define APP_FOC_DEADTIME_ACTIVE (1u)
#include "foc_deadtime_comp.h"
#else
#define APP_FOC_DEADTIME_ACTIVE (0u)
#endif
#if (FOC_HF_DEBUG_SNAPSHOT_ENABLE != 0u)
#include "foc_trace.h"
#endif
#include "foc_observer.h"
#include "foc_startup.h"
#include "foc_protection.h"
#include "foc_svpwm.h"
#include "foc_current_sense.h"
#include "foc_speed_loop.h"
#include "foc_handover.h"
#include "stm32g4xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/*
 * 应用层内部对象。
 *
 * 这些对象都定义为静态全局，只允许本文件直接访问。
 * 外部如果需要读取运行信息，统一通过快照接口获取，
 * 这样可以减少模块间的耦合。
 */
static volatile FOC_StateTypeDef s_state = FOC_STATE_IDLE;
static FOC_CurrentSense_t s_current_sense = {0};
static FOC_Svpwm_t s_svpwm = {0};
#if (APP_FOC_DEADTIME_ACTIVE != 0u)
static FOC_DeadtimeComp_t s_deadtime = {0};
#endif
#if (FOC_HF_DEBUG_SNAPSHOT_ENABLE != 0u)
static FOC_Trace_t s_trace = {0};
#endif
static FOC_CurrentLoopData_t s_current_loop = {0};
static FOC_Startup_t s_startup = {0};
static FOC_ObserverData_t s_observer = {0};
static FOC_Handover_t s_handover = {0};
static FOC_SpeedLoopData_t s_speed_loop = {0};
static uint32_t s_fault_flags = FOC_FAULT_NONE;

/* 中频任务节拍与流程常量。
 *
 * 说明：
 * - 当前阶段仍以电流环 / 开环调试为主；
 * - 因此这里先把“状态推进”和“实时执行”拆开；
 * - 速度环只预留框架，不在当前版本真正接管。
 */
#define APP_FOC_MEDIUM_TASK_PERIOD_MS   (1u)

typedef enum
{
  APP_FOC_THETA_MODE_ALIGN = 0,  /* 固定对齐角。 */
  APP_FOC_THETA_MODE_OPENLOOP,   /* 使用开环轨迹角。 */
  APP_FOC_THETA_MODE_OBSERVER    /* 使用 observer 估算角。 */
} AppFoc_ThetaMode_t;

typedef struct
{
  FOC_StateTypeDef state;              /* 中频状态机给高频侧的目标状态。 */
  AppFoc_ThetaMode_t theta_mode;       /* 本状态期望使用的角度来源。 */
  float id_ref_a;                      /* d 轴电流给定。 */
  float iq_ref_a;                      /* q 轴电流给定。 */
  float speed_ref_mech_rad_s;          /* 机械速度给定，供 RUN/observer 参考。 */
  uint8_t speed_loop_enable;           /* 速度环是否已经允许接管 iq_ref。 */
} AppFoc_Command_t;

typedef struct
{
  uint8_t offset_done;       /* 零点校准是否完成。 */
  uint8_t observer_ready;    /* handover 是否认为 observer 可接管。 */
  uint32_t align_ticks;      /* 对齐阶段已累计的高频 tick。 */
  float mech_freq_hz;        /* 当前开环机械频率。 */
  uint32_t fault_flags;      /* 最近一次故障位图。 */
} AppFoc_Feedback_t;

static AppFoc_Command_t s_cmd = {FOC_STATE_IDLE, APP_FOC_THETA_MODE_ALIGN, 0.0f, 0.0f, 0.0f, 0u};
static volatile AppFoc_Command_t s_cmd_active = {FOC_STATE_IDLE, APP_FOC_THETA_MODE_ALIGN, 0.0f, 0.0f, 0.0f, 0u};
static AppFoc_Feedback_t s_feedback = {0};
static volatile FOC_RuntimeSnapshot_t s_runtime_snapshot;
static volatile uint32_t s_runtime_seq = 0u;
static FOC_FaultSnapshot_t s_fault_snapshot = {0};
static uint32_t s_last_medium_tick_ms = 0u;
static float s_medium_task_dt_sec = APP_FOC_MEDIUM_TASK_PERIOD_MS * 0.001f;

static uint32_t s_d_axis_soft_start_ms = 0u;
static uint8_t s_run_entry_prepared = 0u;
static uint8_t s_last_dt_enable = 0u;
static float s_d_axis_enable_k = 0.0f;
static float s_d_axis_vd_limit_v = FOC_D_AXIS_VD_LIMIT_START_V;

static float app_foc_clampf(float x, float lo, float hi)
{
  if (x > hi)
  {
    return hi;
  }
  if (x < lo)
  {
    return lo;
  }
  return x;
}

static float app_foc_mech_hz_to_rad_s(float mech_hz)
{
  const float two_pi = 6.28318530718f;
  return mech_hz * two_pi;
}

static void app_foc_commit_command_active(void)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_cmd_active = s_cmd;

  if (primask == 0u)
  {
    __enable_irq();
  }
}

static AppFoc_Command_t app_foc_get_active_command(void)
{
  AppFoc_Command_t cmd;

  cmd.state = s_cmd_active.state;
  cmd.theta_mode = s_cmd_active.theta_mode;
  cmd.id_ref_a = s_cmd_active.id_ref_a;
  cmd.iq_ref_a = s_cmd_active.iq_ref_a;
  cmd.speed_ref_mech_rad_s = s_cmd_active.speed_ref_mech_rad_s;
  cmd.speed_loop_enable = s_cmd_active.speed_loop_enable;

  return cmd;
}

static void app_foc_prepare_run_entry(void)
{
  /* OPENLOOP -> RUN 是在中频状态机内部直接改 s_state 的，
   * 如果只依赖下一拍的 state_changed，会错过 RUN 的入口初始化。
   * 因此这里把 RUN 首拍需要的准备收口成一个显式入口函数。 */
  FOC_SpeedLoopSnapshot_t speed_snapshot;

  FOC_SpeedLoopPrepareRunEntry(&s_speed_loop,
                               s_observer.speed_rad_s,
                               FOC_Startup_GetElecSpeedRadPerSec(&s_startup));

  /* RUN 入口不再清电流环 PI。
   * Id/Iq 双环已经在 OPENLOOP 阶段逐步接管，RUN 只切换角度来源和速度环给定。 */
  s_run_entry_prepared = 1u;

  FOC_SpeedLoopGetSnapshot(&s_speed_loop, &speed_snapshot);
  BSP_KEY_SetSpeedTargetHz(speed_snapshot.speed_ref_mech_rad_s * 0.15915494309f);
}

static void app_foc_d_axis_soft_reset(void)
{
  s_d_axis_soft_start_ms = HAL_GetTick();
  s_d_axis_enable_k = 0.0f;
  s_d_axis_vd_limit_v = FOC_D_AXIS_VD_LIMIT_START_V;
}

static void app_foc_d_axis_soft_update(void)
{
  uint32_t elapsed_ms;
  float k;

  if (s_d_axis_soft_start_ms == 0u)
  {
    app_foc_d_axis_soft_reset();
  }

  elapsed_ms = HAL_GetTick() - s_d_axis_soft_start_ms;
  k = (float)elapsed_ms / (float)FOC_D_AXIS_SOFT_RAMP_MS;
  s_d_axis_enable_k = app_foc_clampf(k, 0.0f, 1.0f);
  s_d_axis_vd_limit_v = FOC_D_AXIS_VD_LIMIT_START_V +
      ((FOC_D_AXIS_VD_LIMIT_END_V - FOC_D_AXIS_VD_LIMIT_START_V) * s_d_axis_enable_k);
}

/*
 * 控制驱动器使能引脚。
 *
 * enable = 1：允许功率级导通；
 * enable = 0：关闭功率级输出。
 */
static void app_foc_drive_enable(uint8_t enable)
{
  HAL_GPIO_WritePin(M1_EN_DRIVER_GPIO_Port,
                    M1_EN_DRIVER_Pin,
                    enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
 * 刷新本拍采样结果。
 *
 * 这里先把当前 SVPWM 扇区传给采样模块，
 * 再由采样模块决定两相可信关系和电流重构方式。
 */
static void app_foc_current_sense_update(void)
{
  uint8_t fb_req = 0u;

  /*
   * s_svpwm.svpwm_sector 表示当前 ADC 数据应使用的解码扇区。
   * 采样模块只根据这个扇区选择可信两相和重构第三相。
   */
  FOC_CurrentSense_SetSampleSector(&s_current_sense, s_svpwm.svpwm_sector);

  /*
   * hold-last fallback 只在 OPENLOOP/RUN 中启用。
   * ALIGN 阶段电流环正在建立初始磁场，若冻结电流反馈，PI 容易持续推高电压。
   */
  if ((s_state == FOC_STATE_OPENLOOP) || (s_state == FOC_STATE_RUN))
  {
    fb_req = s_svpwm.adc_trigger_fallback;
  }

  FOC_CurrentSense_SetFallbackRequest(&s_current_sense, fb_req);
  FOC_CurrentSense_Update(&s_current_sense);
}

/*
 * 强制三相 PWM 回到中点占空比。
 *
 * 典型使用场景：
 * - 上电初始化；
 * - 零点校准；
 * - 空闲状态；
 * - 故障停机后回中点。
 */
static void app_foc_force_mid_output(void)
{
  FOC_Svpwm_SetTim1Mid(&s_svpwm);
}

static void app_foc_shutdown_outputs(void)
{
  FOC_Svpwm_SetTim1Mid(&s_svpwm);
  app_foc_drive_enable(0u);
}


static void app_foc_apply_voltage_command(float *valpha, float *vbeta, uint8_t dt_enable)
{
  float valpha_cmd;
  float vbeta_cmd;

  if ((valpha == 0) || (vbeta == 0))
  {
    return;
  }

  /* 所有控制路径最终都汇到这里：
   * valpha/vbeta -> 可选死区补偿 -> SVPWM -> TIM1 CCR。
   * 这样可以保证不同状态下的 PWM 输出入口一致。
   */
  valpha_cmd = *valpha;
  vbeta_cmd = *vbeta;
#if (APP_FOC_DEADTIME_ACTIVE != 0u)
  s_last_dt_enable = dt_enable;

  if (dt_enable != 0u)
  {
    FOC_DeadtimeComp_Apply(&s_deadtime,
                           s_current_sense.vbus_v,
                           s_current_sense.iu_a,
                           s_current_sense.iv_a,
                           s_current_sense.iw_a,
                           valpha_cmd,
                           vbeta_cmd,
                           &valpha_cmd,
                           &vbeta_cmd);
  }
  else
  {
    FOC_DeadtimeComp_Reset(&s_deadtime);
  }
#else
  (void)dt_enable;
  s_last_dt_enable = 0u;
#endif

  FOC_Svpwm_ApplySvpwm(&s_svpwm, valpha_cmd, vbeta_cmd, s_current_sense.vbus_v);
  *valpha = valpha_cmd;
  *vbeta = vbeta_cmd;
}


/*
 * 清空非运行阶段的电流环调试量。
 *
 * 这些状态下不会真正向电机输出 dq 控制指令，
 * 因此把对外可见的电流环量统一清零，避免日志残留上一状态数据。
 */
static void app_foc_clear_current_loop_debug(void)
{
  s_current_loop.id_ref_a = 0.0f;
  s_current_loop.iq_ref_a = 0.0f;
  s_current_loop.id_meas_a = 0.0f;
  s_current_loop.iq_meas_a = 0.0f;
  s_current_loop.vd_pi_v = 0.0f;
  s_current_loop.vq_pi_v = 0.0f;
  s_current_loop.vd_ff_v = 0.0f;
  s_current_loop.vq_ff_v = 0.0f;
  s_current_loop.vd_ff_raw_v = 0.0f;
  s_current_loop.vq_ff_raw_v = 0.0f;
  s_current_loop.bemf_ff_blend = 0.0f;
  s_current_loop.decouple_ff_blend = 0.0f;
  s_current_loop.decouple_ff_delay_ticks = 0u;
  s_current_loop.ff_enable = 0u;
  s_current_loop.vd_cmd_v = 0.0f;
  s_current_loop.vq_cmd_v = 0.0f;
  s_current_loop.valpha_cmd_v = 0.0f;
  s_current_loop.vbeta_cmd_v = 0.0f;
  s_current_loop.theta_ctrl_rad = 0.0f;
  s_last_dt_enable = 0u;
}

static void app_foc_latch_fault_snapshot(void)
{
  if (s_fault_snapshot.valid != 0u)
  {
    return;
  }

  s_fault_snapshot.valid = 1u;
  s_fault_snapshot.state = s_state;
  s_fault_snapshot.fault_flags = s_fault_flags;

  s_fault_snapshot.id_ref_a = s_current_loop.id_ref_a;
  s_fault_snapshot.iq_ref_a = s_current_loop.iq_ref_a;
  s_fault_snapshot.id_meas_a = s_current_loop.id_meas_a;
  s_fault_snapshot.iq_meas_a = s_current_loop.iq_meas_a;

  s_fault_snapshot.vd_v = s_current_loop.vd_cmd_v;
  s_fault_snapshot.vq_v = s_current_loop.vq_cmd_v;
  s_fault_snapshot.valpha_v = s_current_loop.valpha_cmd_v;
  s_fault_snapshot.vbeta_v = s_current_loop.vbeta_cmd_v;

  s_fault_snapshot.theta_ctrl_rad = s_current_loop.theta_ctrl_rad;
  s_fault_snapshot.theta_open_rad = s_startup.theta_e_rad;
  s_fault_snapshot.theta_obs_rad = s_observer.theta_rad;

  s_fault_snapshot.open_speed_rad_s = FOC_Startup_GetElecSpeedRadPerSec(&s_startup);
  s_fault_snapshot.obs_speed_rad_s = s_observer.speed_rad_s;
  {
    FOC_SpeedLoopSnapshot_t speed_snapshot;
    FOC_SpeedLoopGetSnapshot(&s_speed_loop, &speed_snapshot);
    s_fault_snapshot.speed_ref_mech_rad_s = speed_snapshot.speed_ref_mech_rad_s;
    s_fault_snapshot.speed_fdb_mech_rad_s = speed_snapshot.speed_fdb_mech_rad_s;
  }

  s_fault_snapshot.vbus_v = s_current_sense.vbus_v;

  s_fault_snapshot.svpwm_sector = s_svpwm.svpwm_sector;
  s_fault_snapshot.fallback_request = s_current_sense.fallback_request;
  s_fault_snapshot.used_hold_last_current = s_current_sense.used_hold_last_current;
  s_fault_snapshot.fallback_hold_count = s_current_sense.fallback_hold_count;
}

/*
 * 统一进入故障状态。
 *
 * App 层统一执行停机动作：
 * - 记录兜底故障位；
 * - 锁存首次故障现场；
 * - PWM 回中点；
 * - 关闭驱动；
 * - 状态锁到 FAULT。
 */
static void app_foc_enter_fault(void)
{
  if (s_fault_flags == FOC_FAULT_NONE)
  {
    /* 若外部主动停机走到这里，没有具体保护原因时保留一个兜底标记。 */
    s_fault_flags = 0x80000000u;
  }
  app_foc_latch_fault_snapshot();
  app_foc_shutdown_outputs();
  s_state = FOC_STATE_FAULT;
}

/*
 * 填充运行快照。
 *
 * 目的：
 * 把应用层内部分散在多个模块中的关键运行量统一打包，
 * 让低频调试任务一次性取走，避免外部堆大量 getter。
 */
static void app_foc_fill_runtime_snapshot(FOC_RuntimeSnapshot_t *snapshot)
{
  if (snapshot == 0)
  {
    return;
  }

  /* PWM / 扇区信息。 */
  snapshot->state = s_state;
  snapshot->fault_flags = s_fault_flags;
  snapshot->svpwm_sector = s_svpwm.svpwm_sector;

  /* 开环信息。 */
  snapshot->startup.freq_hz = s_startup.mech_freq_hz;
  snapshot->startup.theta_e_rad = s_startup.theta_e_rad;

  /* 观测器信息。 */
  snapshot->observer.flux_theta_rad = FOC_ObserverGetFluxThetaRad(&s_observer);
  snapshot->observer.theta_rad = s_observer.theta_rad;
  snapshot->observer.speed_rad_s = s_observer.speed_rad_s;
  snapshot->observer.angle_err_deg = s_handover.angle_err_deg;
  snapshot->observer.vbus_v = s_current_sense.vbus_v;
  snapshot->observer.locked = s_handover.locked;
  snapshot->observer.pll_err = s_observer.flux_obs.pll_err;
  snapshot->observer.pll_integral = s_observer.flux_obs.pll_integral;
  snapshot->observer.pll_speed_raw_rad_s = s_observer.flux_obs.pll_speed_raw_rad_s;
  snapshot->observer.pll_speed_filt_rad_s = s_observer.flux_obs.pll_speed_filt_rad_s;
  snapshot->observer.flux_alpha = s_observer.flux_obs.Flux_alpha_O;
  snapshot->observer.flux_beta = s_observer.flux_obs.Flux_beta_O;
  snapshot->observer.flux_mag = s_observer.flux_obs.flux_mag;
  snapshot->observer.speed_limit_hit = s_observer.flux_obs.speed_limit_hit;
  snapshot->observer.integral_limit_hit = s_observer.flux_obs.integral_limit_hit;
  FOC_ObserverGetInputs(&s_observer,
                        &snapshot->observer.valpha_v,
                        &snapshot->observer.vbeta_v,
                        &snapshot->observer.ialpha_a,
                        &snapshot->observer.ibeta_a);

  /* 采样信息。 */
  snapshot->current_sense.vbus_v = s_current_sense.vbus_v;
  snapshot->current_sense.iu_a = s_current_sense.iu_a;
  snapshot->current_sense.iv_a = s_current_sense.iv_a;
  snapshot->current_sense.iw_a = s_current_sense.iw_a;
  snapshot->current_sense.iu_offset_v = s_current_sense.iu_offset_v;
  snapshot->current_sense.iv_offset_v = s_current_sense.iv_offset_v;
  snapshot->current_sense.iw_offset_v = s_current_sense.iw_offset_v;
  snapshot->current_sense.raw_u = s_current_sense.raw_u;
  snapshot->current_sense.raw_v = s_current_sense.raw_v;
  snapshot->current_sense.raw_w = s_current_sense.raw_w;
  snapshot->current_sense.adc1_j1_raw = s_current_sense.adc1_j1_raw;
  snapshot->current_sense.adc1_j2_raw = s_current_sense.adc1_j2_raw;
  snapshot->current_sense.adc2_j1_raw = s_current_sense.adc2_j1_raw;
  snapshot->current_sense.adc2_j2_raw = s_current_sense.adc2_j2_raw;
  snapshot->current_sense.adc2_j3_raw = s_current_sense.adc2_j3_raw;
  snapshot->current_sense.trusted_pair = s_current_sense.trusted_current_pair;
  snapshot->current_sense.fallback_request = s_current_sense.fallback_request;
  snapshot->current_sense.used_hold_last_current = s_current_sense.used_hold_last_current;
  snapshot->current_sense.fallback_hold_count = s_current_sense.fallback_hold_count;

  /* 电流环信息。 */
  snapshot->current_loop.id_ref_a = s_current_loop.id_ref_a;
  snapshot->current_loop.iq_ref_a = s_current_loop.iq_ref_a;
  snapshot->current_loop.id_meas_a = s_current_loop.id_meas_a;
  snapshot->current_loop.iq_meas_a = s_current_loop.iq_meas_a;
  snapshot->current_loop.vd_pi_v = s_current_loop.vd_pi_v;
  snapshot->current_loop.vq_pi_v = s_current_loop.vq_pi_v;
  snapshot->current_loop.vd_ff_v = s_current_loop.vd_ff_v;
  snapshot->current_loop.vq_ff_v = s_current_loop.vq_ff_v;
  snapshot->current_loop.vd_ff_raw_v = s_current_loop.vd_ff_raw_v;
  snapshot->current_loop.vq_ff_raw_v = s_current_loop.vq_ff_raw_v;
  snapshot->current_loop.bemf_ff_blend = s_current_loop.bemf_ff_blend;
  snapshot->current_loop.decouple_ff_blend = s_current_loop.decouple_ff_blend;
  snapshot->current_loop.ff_enable = s_current_loop.ff_enable;
  snapshot->current_loop.vd_cmd_v = s_current_loop.vd_cmd_v;
  snapshot->current_loop.vq_cmd_v = s_current_loop.vq_cmd_v;
  snapshot->current_loop.valpha_cmd_v = s_current_loop.valpha_cmd_v;
  snapshot->current_loop.vbeta_cmd_v = s_current_loop.vbeta_cmd_v;
  snapshot->current_loop.theta_ctrl_rad = s_current_loop.theta_ctrl_rad;
  snapshot->current_loop.d_axis_enable_k = s_d_axis_enable_k;
  snapshot->current_loop.run_iq_only = 0u;
  snapshot->current_loop.dt_enable = s_last_dt_enable;

  /* 速度环信息。 */
  FOC_SpeedLoopGetSnapshot(&s_speed_loop, &snapshot->speed_loop);

  /* 开环到观测器切换信息。 */
  snapshot->handover.state = (uint8_t)s_handover.state;
  snapshot->handover.observer_ready = s_handover.observer_ready;
  snapshot->handover.ready_now = s_handover.ready_now;
  snapshot->handover.locked = s_handover.locked;
  snapshot->handover.hold_ticks = s_handover.hold_ticks;
  snapshot->handover.lock_hold_ticks = s_handover.lock_hold_ticks;
  snapshot->handover.blend_ticks = s_handover.blend_ticks;
  snapshot->handover.blend_k = s_handover.blend_k;
  snapshot->handover.theta_open_rad = s_startup.theta_e_rad;
  snapshot->handover.theta_obs_rad = s_observer.theta_rad;
  snapshot->handover.theta_ctrl_rad = s_current_loop.theta_ctrl_rad;
  snapshot->handover.angle_err_deg = s_handover.angle_err_deg;
  snapshot->handover.open_speed_rad_s = FOC_Startup_GetElecSpeedRadPerSec(&s_startup);
  snapshot->handover.obs_speed_rad_s = s_observer.speed_rad_s;
  snapshot->handover.speed_err_rad_s = s_handover.speed_err_rad_s;
  snapshot->handover.fail_reason = s_handover.fail_reason;
  snapshot->handover.last_blend_reset_reason = s_handover.last_blend_reset_reason;
  snapshot->handover.blend_reset_count = s_handover.blend_reset_count;
  snapshot->handover.last_blend_reset_angle_err_deg = s_handover.last_blend_reset_angle_err_deg;
}

static void app_foc_update_runtime_snapshot(void)
{
  FOC_RuntimeSnapshot_t tmp;

  app_foc_fill_runtime_snapshot(&tmp);

  s_runtime_seq++;
  s_runtime_snapshot = tmp;
  s_runtime_seq++;
}

/*
 * 对齐阶段高频执行逻辑。
 *
 * 处理流程：
 * 1. 先把 PWM 回到中点，保证从安全状态开始；
 * 2. 第一次进入时打开驱动；
 * 3. 刷新电流/母线采样并做保护检查；
 * 4. 固定角度运行 d 轴电流闭环，把转子吸到预期电角位置；
 * 5. 对齐时间到后切到 OPENLOOP。
 */
static void app_foc_run_align(void)
{
  ClarkePark_AlphaBeta_t vab = {0.0f, 0.0f};
  ClarkePark_DQ_t vdq = {0.0f, 0.0f};
  float valpha = 0.0f;
  float vbeta = 0.0f;

  if (s_startup.align_ticks == 0u)
  {
    app_foc_drive_enable(1u);
  }

  app_foc_current_sense_update();
  s_fault_flags = FOC_ProtectionCheckFault(&s_current_sense);
  if (s_fault_flags != FOC_FAULT_NONE)
  {
    app_foc_enter_fault();
    return;
  }

  /* 对齐阶段改为：
   * - 电角固定在 0；
   * - 通过 d 轴电流闭环建立静止磁场；
   * - q 轴给 0，避免对齐阶段引入额外转矩。
   *
   * 这样比固定电压对齐更柔和，也更符合当前主线已经切到电流环的结构。
   */
  s_startup.theta_e_rad = 0.0f;
  FOC_CurrentLoopRunWithRef(&s_current_loop,
                            &s_current_sense,
                            s_startup.theta_e_rad,
                            FOC_ALIGN_ID_REF_A,
                            0.0f,
                            &valpha,
                            &vbeta);
  /* 对齐阶段只保留 d 轴磁场，直接把 q 轴电压清零，避免额外转矩导致持续抖动。 */
  vdq.d = s_current_loop.vd_cmd_v;
  vdq.q = 0.0f;
  ClarkePark_InvPark(&vdq, s_startup.theta_e_rad, &vab);
  s_current_loop.vq_cmd_v = 0.0f;
  s_current_loop.valpha_cmd_v = vab.alpha;
  s_current_loop.vbeta_cmd_v = vab.beta;
  valpha = s_current_loop.valpha_cmd_v;
  vbeta = s_current_loop.vbeta_cmd_v;
  app_foc_apply_voltage_command(&valpha, &vbeta, FOC_DT_COMP_ENABLE_ALIGN);

  s_startup.align_ticks++;
  /* 对齐结束后的状态跳转交给中频任务处理。
   * 高频这里只负责持续输出对齐磁场并累计 ticks。 */
}

/*
 * 开环阶段高频执行逻辑。
 *
 * 当前流程：
 * 1. 刷新采样并做保护检查；
 * 2. 推进开环角度/频率；
 * 3. 更新开环到观测器切换器；
 * 4. 选出本拍控制角；
 * 5. 根据配置运行开环电流模式 / dq 电压测试 / V/F 模式；
 * 6. 更新 SVPWM；
 * 7. 后台刷新观测器。
 */
static void app_foc_run_openloop(void)
{
  const AppFoc_Command_t cmd = app_foc_get_active_command();
  float theta_ctrl;
  float valpha = 0.0f;
  float vbeta = 0.0f;

  app_foc_current_sense_update();
  s_fault_flags = FOC_ProtectionCheckFault(&s_current_sense);
  if (s_fault_flags != FOC_FAULT_NONE)
  {
    app_foc_enter_fault();
    return;
  }

  FOC_Startup_Update(&s_startup);
  FOC_Handover_Update(&s_handover, &s_startup, &s_observer);
  theta_ctrl = FOC_Handover_SelectTheta(&s_handover, &s_startup, &s_observer);

#if (FOC_OPENLOOP_CTRL_MODE == FOC_CTRL_MODE_OPENLOOP_CURRENT)
  /* OPENLOOP 开始即使用 Id/Iq 双电流环。
   * d 轴通过 gain + vd_limit 软释放，避免后续 RUN 阶段突然接管。 */
  app_foc_d_axis_soft_update();
  FOC_CurrentLoopRunDSoft(&s_current_loop,
                          &s_current_sense,
                          theta_ctrl,
                          cmd.id_ref_a,
                          cmd.iq_ref_a,
                          s_d_axis_enable_k,
                          s_d_axis_vd_limit_v,
                          &valpha,
                          &vbeta);
#elif (FOC_OPENLOOP_CTRL_MODE == FOC_CTRL_MODE_OPENLOOP_DQ_VOLT_TEST)
  FOC_CurrentLoopRunOpenLoopVoltTest(&s_current_loop, &s_current_sense, theta_ctrl, &valpha, &vbeta);
#else
  FOC_CurrentLoopRunOpenLoopVf(&s_current_loop, &s_current_sense, theta_ctrl, &valpha, &vbeta);
#endif

  app_foc_apply_voltage_command(&valpha, &vbeta, FOC_DT_COMP_ENABLE_OPENLOOP);

  FOC_ObserverUpdate(&s_observer,
                     &s_current_sense,
                     valpha,
                     vbeta);
}

static void app_foc_sync_feedback_from_modules(void)
{
  s_feedback.offset_done = (s_current_sense.offset_samples >= FOC_CURRENT_OFFSET_CALIB_SAMPLES) ? 1u : 0u;
  s_feedback.observer_ready = s_handover.observer_ready;
  s_feedback.align_ticks = s_startup.align_ticks;
  s_feedback.mech_freq_hz = s_startup.mech_freq_hz;
  s_feedback.fault_flags = s_fault_flags;
}

static void app_foc_medium_reset_command_defaults(void)
{
  s_cmd.state = s_state;
  s_cmd.theta_mode = APP_FOC_THETA_MODE_OPENLOOP;
  s_cmd.id_ref_a = FOC_ID_REF_A;
  s_cmd.iq_ref_a = FOC_IQ_REF_A;
  {
    FOC_SpeedLoopSnapshot_t speed_snapshot;
    FOC_SpeedLoopGetSnapshot(&s_speed_loop, &speed_snapshot);
    s_cmd.speed_ref_mech_rad_s = speed_snapshot.speed_ref_mech_rad_s;
  }
  s_cmd.speed_loop_enable = 0u;
}

static void app_foc_medium_handle_state_machine(void)
{
  static FOC_StateTypeDef s_prev_state = FOC_STATE_IDLE;
  const FOC_StateTypeDef state_entry = s_state;
  const float inv_ctrl_period_sec = 1.0f / FOC_TS_SEC;
  const uint32_t align_ticks_target =
      (uint32_t)((FOC_ALIGN_TIME_MS * 0.001f) * inv_ctrl_period_sec);
  const uint8_t state_changed = (state_entry != s_prev_state) ? 1u : 0u;

  app_foc_sync_feedback_from_modules();

  /* 先给命令一个安全默认值。
   * 各状态只覆盖自己真正需要改的字段，
   * 这样能避免某个状态忘记赋值时残留上一状态命令。 */
  app_foc_medium_reset_command_defaults();
  s_cmd.state = s_state;

  switch (s_state)
  {
    case FOC_STATE_IDLE:
      /* 待机阶段：
       * - 电机不运行；
       * - 命令保持安全值；
       * - 启动入口由 AppFoc_Start() 直接把状态切到 OFFSET_CALIB。 */
      s_cmd.theta_mode = APP_FOC_THETA_MODE_OPENLOOP;
      s_cmd.speed_loop_enable = 0u;
      s_cmd.id_ref_a = 0.0f;
      s_cmd.iq_ref_a = 0.0f;
      s_cmd.speed_ref_mech_rad_s = 0.0f;

      if (state_changed != 0u)
      {
        /* 刚进入 IDLE 时做一次性清理，
         * 不要每个 1 ms 周期都 reset，避免把状态一直洗掉。 */
        FOC_Handover_Reset(&s_handover);
        FOC_CurrentLoopReset(&s_current_loop);
        FOC_Startup_ResetAlign(&s_startup);
      }
      break;

    case FOC_STATE_OFFSET_CALIB:
      /* 零点校准阶段：
       * 高频里只负责累计 offset 样本；
       * 中频里根据 offset_done 决定是否进入 ALIGN。 */
      s_cmd.theta_mode = APP_FOC_THETA_MODE_ALIGN;
      s_cmd.speed_loop_enable = 0u;

      if (state_changed != 0u)
      {
        /* 进入零点校准时，把后续运行相关模块先清一次。 */
        FOC_Handover_Reset(&s_handover);
        FOC_CurrentLoopReset(&s_current_loop);
        FOC_Startup_ResetAlign(&s_startup);
      }

      if (s_feedback.offset_done != 0u)
      {
        FOC_Startup_ResetAlign(&s_startup);
        s_state = FOC_STATE_ALIGN;
      }
      break;

    case FOC_STATE_ALIGN:
      /* 对齐阶段：
       * 高频里持续输出固定对齐磁场；
       * 中频里只判断对齐时间是否已经足够。 */
      s_cmd.theta_mode = APP_FOC_THETA_MODE_ALIGN;
      s_cmd.speed_loop_enable = 0u;

      if (state_changed != 0u)
      {
        FOC_Startup_ResetAlign(&s_startup);
      }

      if (s_feedback.align_ticks >= align_ticks_target)
      {
        /* 对齐结束后，把开环、电流环、接管器清一次，
         * 保证进入 OPENLOOP 时从干净状态开始。 */
        FOC_Startup_ResetRun(&s_startup);
        FOC_Handover_Reset(&s_handover);
        FOC_CurrentLoopReset(&s_current_loop);
        app_foc_d_axis_soft_reset();
        s_run_entry_prepared = 0u;
        s_state = FOC_STATE_OPENLOOP;
      }
      break;

    case FOC_STATE_OPENLOOP:
      /* 开环阶段：
       * 高频里会一边推进开环角，一边后台更新 observer；
       * 中频只关心 observer 是否已经真正接管成功。 */
      s_cmd.theta_mode = APP_FOC_THETA_MODE_OPENLOOP;
      s_cmd.speed_loop_enable = 0u;

      if (state_changed != 0u)
      {
        FOC_Handover_Reset(&s_handover);
        FOC_CurrentLoopReset(&s_current_loop);
        FOC_Startup_ResetRun(&s_startup);
        app_foc_d_axis_soft_reset();
        s_run_entry_prepared = 0u;
      }

      if (s_handover.state == FOC_HANDOVER_STATE_OBSERVER)
      {
        /* 先准备 RUN 入口，再切换状态，避免 RUN 首拍 Iq-only hold 未生效。 */
        app_foc_prepare_run_entry();
        s_state = FOC_STATE_RUN;
      }
      break;

    case FOC_STATE_RUN:
      /* 运行阶段：
       * - observer 角度完全接管；
       * - 中频速度环输出 iq_ref；
       * - 当前调试版本先只允许正扭矩，不做制动。 */
      s_cmd.theta_mode = APP_FOC_THETA_MODE_OBSERVER;
      s_cmd.id_ref_a = 0.0f;

      if ((state_changed != 0u) && (s_run_entry_prepared == 0u))
      {
        /* 兜底保留 RUN 入口初始化；OPENLOOP->RUN 正常路径会提前执行。 */
        app_foc_prepare_run_entry();
      }

      {
        FOC_SpeedLoopSnapshot_t speed_snapshot;
        FOC_SpeedLoopStep(&s_speed_loop, s_observer.speed_rad_s, s_medium_task_dt_sec);
        FOC_SpeedLoopGetSnapshot(&s_speed_loop, &speed_snapshot);
        s_cmd.speed_loop_enable = FOC_SpeedLoopIsEnabled(&s_speed_loop);
        s_cmd.speed_ref_mech_rad_s = speed_snapshot.speed_ref_mech_rad_s;
        s_cmd.iq_ref_a = speed_snapshot.iq_ref_cmd_a;
      }

      /* 当前版本仍保留最小回退：若接管状态丢失，则退回 OPENLOOP。 */
      if (s_handover.state != FOC_HANDOVER_STATE_OBSERVER)
      {
        FOC_Startup_ResetRun(&s_startup);
        FOC_Handover_Reset(&s_handover);
        app_foc_d_axis_soft_reset();
        s_run_entry_prepared = 0u;
        s_state = FOC_STATE_OPENLOOP;
      }
      break;

    case FOC_STATE_FAULT:
      /* 故障阶段：
       * 这里只保持安全命令；
       * 真正的停机收口由 App 层故障入口统一执行。 */
      s_cmd.theta_mode = APP_FOC_THETA_MODE_OPENLOOP;
      s_cmd.speed_loop_enable = 0u;
      s_cmd.id_ref_a = 0.0f;
      s_cmd.iq_ref_a = 0.0f;
      s_cmd.speed_ref_mech_rad_s = 0.0f;

      if (state_changed != 0u)
      {
        FOC_CurrentLoopReset(&s_current_loop);
        FOC_Handover_Reset(&s_handover);
      }
      break;

    default:
      s_state = FOC_STATE_IDLE;
      break;
  }

  s_cmd.state = s_state;
  app_foc_commit_command_active();
  s_prev_state = s_state;
}

static void app_foc_run_run(void)
{
  const AppFoc_Command_t cmd = app_foc_get_active_command();
  float theta_ctrl = 0.0f;
  float valpha = 0.0f;
  float vbeta = 0.0f;

  app_foc_current_sense_update();
  s_fault_flags = FOC_ProtectionCheckFault(&s_current_sense);
  if (s_fault_flags != FOC_FAULT_NONE)
  {
    app_foc_enter_fault();
    return;
  }

  /* RUN 阶段不再推进开环角，直接使用 observer 角闭环运行。
   * Id/Iq 双电流环已经在 OPENLOOP 阶段逐步接管，这里不再做 Iq-only hold。 */
  theta_ctrl = FOC_ObserverGetThetaRad(&s_observer);

  app_foc_d_axis_soft_update();
  FOC_CurrentLoopRunDSoftFeedForward(&s_current_loop,
                                     &s_current_sense,
                                     theta_ctrl,
                                     cmd.id_ref_a,
                                     cmd.iq_ref_a,
                                     s_d_axis_enable_k,
                                     s_d_axis_vd_limit_v,
                                     FOC_ObserverGetSpeedRadPerSec(&s_observer),
                                     s_current_sense.vbus_v,
                                     &valpha,
                                     &vbeta);

  app_foc_apply_voltage_command(&valpha, &vbeta, FOC_DT_COMP_ENABLE_RUN);

  /* observer 仍在后台更新；接管判定统一由 handover 模块负责。 */
  FOC_ObserverUpdate(&s_observer,
                     &s_current_sense,
                     valpha,
                     vbeta);
}

void AppFoc_Init(void)
{
  /* 初始化所有控制对象，并回到一个安全静止状态。 */
  s_state = FOC_STATE_IDLE;
#if (APP_DWT_TIMING_ENABLE != 0u)
  AppFocTiming_Init();
#endif

  FOC_CurrentSense_Init(&s_current_sense);
  FOC_Svpwm_Init(&s_svpwm);
#if (APP_FOC_DEADTIME_ACTIVE != 0u)
  FOC_DeadtimeComp_Init(&s_deadtime);
#endif
#if (FOC_HF_DEBUG_SNAPSHOT_ENABLE != 0u)
  FOC_Trace_Init(&s_trace);
#endif
  FOC_CurrentLoopInit(&s_current_loop);
  FOC_Startup_Init(&s_startup);
  FOC_ObserverInit(&s_observer);
  FOC_Handover_Init(&s_handover);
  FOC_SpeedLoopInit(&s_speed_loop);
  app_foc_medium_reset_command_defaults();
  app_foc_commit_command_active();
  s_feedback.offset_done = 0u;
  s_feedback.observer_ready = 0u;
  s_feedback.align_ticks = 0u;
  s_feedback.mech_freq_hz = 0.0f;
  s_feedback.fault_flags = 0u;
  s_fault_flags = FOC_FAULT_NONE;
  AppFoc_ClearFaultSnapshot();
  s_last_medium_tick_ms = 0u;
  app_foc_d_axis_soft_reset();
  s_run_entry_prepared = 0u;
  s_runtime_seq = 0u;
  app_foc_update_runtime_snapshot();

  app_foc_force_mid_output();
  app_foc_drive_enable(0u);
}

void AppFoc_Start(void)
{
  /* 启动三相 PWM 和互补输出。 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  /* 启动 TIM1 计数器本体；高频 FOC 由 ADC2 注入完成中断触发，不再启用 TIM1 Update 中断。 */
  HAL_TIM_Base_Start(&htim1);

  /* 启动 ADC 常规/注入转换。 */
  HAL_ADC_Start(&hadc1);
  FOC_CurrentSense_SetAdc1Ready(&s_current_sense, 1u);
  HAL_ADCEx_InjectedStart(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc2);

  /* 每次启动都要把内部状态重新清空。 */
  FOC_ObserverReset(&s_observer);
#if (APP_FOC_DEADTIME_ACTIVE != 0u)
  FOC_DeadtimeComp_Reset(&s_deadtime);
#endif
  FOC_Handover_Reset(&s_handover);
  FOC_CurrentLoopReset(&s_current_loop);
  FOC_Startup_ResetAlign(&s_startup);
  FOC_Startup_ResetRun(&s_startup);

  s_current_sense.offset_samples = 0u;
  s_current_sense.offset_sum_u = 0.0f;
  s_current_sense.offset_sum_v = 0.0f;
  s_current_sense.offset_sum_w = 0.0f;
  FOC_CurrentSense_ResetRuntime(&s_current_sense);

  /* 启动后先保持中点输出并关闭驱动，等待零点校准。 */
  app_foc_medium_reset_command_defaults();
  app_foc_commit_command_active();
  s_feedback.offset_done = 0u;
  s_feedback.observer_ready = 0u;
  s_feedback.align_ticks = 0u;
  s_feedback.mech_freq_hz = 0.0f;
  s_feedback.fault_flags = 0u;
  s_fault_flags = FOC_FAULT_NONE;
  AppFoc_ClearFaultSnapshot();
#if (APP_DWT_TIMING_ENABLE != 0u)
  AppFoc_ResetHfTimingStats();
#endif
  FOC_SpeedLoopReset(&s_speed_loop, 0.0f);
  app_foc_d_axis_soft_reset();
  s_run_entry_prepared = 0u;
  s_last_medium_tick_ms = HAL_GetTick();
  app_foc_force_mid_output();
  app_foc_drive_enable(0u);
  s_state = FOC_STATE_OFFSET_CALIB;
  app_foc_update_runtime_snapshot();
}

void AppFoc_Stop(void)
{
  /* Normal stop: shut outputs down without creating a fault snapshot. */
  app_foc_shutdown_outputs();

  HAL_ADCEx_InjectedStop(&hadc1);
  HAL_ADCEx_InjectedStop_IT(&hadc2);
  HAL_ADC_Stop(&hadc1);

  HAL_TIM_Base_Stop(&htim1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

  FOC_CurrentSense_SetAdc1Ready(&s_current_sense, 0u);
  s_state = FOC_STATE_IDLE;
  s_fault_flags = FOC_FAULT_NONE;
#if (APP_FOC_DEADTIME_ACTIVE != 0u)
  FOC_DeadtimeComp_Reset(&s_deadtime);
#endif
  app_foc_medium_reset_command_defaults();
  app_foc_commit_command_active();
  app_foc_update_runtime_snapshot();
}

void AppFoc_MediumFreqTask(void)
{
  const uint32_t now_ms = HAL_GetTick();
  const uint32_t elapsed_ms = now_ms - s_last_medium_tick_ms;

  if (elapsed_ms < APP_FOC_MEDIUM_TASK_PERIOD_MS)
  {
    return;
  }

  s_medium_task_dt_sec = (float)elapsed_ms * 0.001f;
  s_last_medium_tick_ms = now_ms;
  app_foc_medium_handle_state_machine();

  /* 运行快照放在中频任务刷新，避免在 16kHz ADC 中断里做大结构体拷贝。 */
  app_foc_update_runtime_snapshot();
}

void AppFoc_HighFreqTask(void)
{
#if (APP_DWT_TIMING_ENABLE != 0u)
  const uint32_t hf_start_cycles = AppFocTiming_GetCycle();
#endif

  /*
   * 高频状态机只负责“这一拍该跑什么”。
   * 复杂的外部命令和低频业务不放在这里，
   * 以保证中断路径尽量短、尽量稳定。
   */
  switch (s_state)
  {
    case FOC_STATE_OFFSET_CALIB:
    {
      FOC_StateTypeDef next_state = s_state;

      app_foc_force_mid_output();
      app_foc_clear_current_loop_debug();
      app_foc_drive_enable(0u);
      FOC_CurrentSense_RunOffsetCalib(&s_current_sense, &next_state);

      /* 高频里只产生“零点校准完成”这一事实，
       * 真正切到 ALIGN 交给中频状态机。 */
      if (next_state == FOC_STATE_ALIGN)
      {
        s_feedback.offset_done = 1u;
      }
      break;
    }

    case FOC_STATE_ALIGN:
      app_foc_run_align();
      break;

    case FOC_STATE_OPENLOOP:
      app_foc_run_openloop();
      break;

    case FOC_STATE_RUN:
      app_foc_run_run();
      break;

    case FOC_STATE_FAULT:
      app_foc_clear_current_loop_debug();
      app_foc_enter_fault();
      break;

    case FOC_STATE_IDLE:
    default:
      app_foc_force_mid_output();
      app_foc_clear_current_loop_debug();
      break;
  }

#if (FOC_HF_DEBUG_SNAPSHOT_ENABLE != 0u)
  /* 高频采样/PWM细节快照默认关闭；排查采样时再打开。 */
  FOC_Trace_UpdateSnapshot(&s_trace, &s_current_sense, &s_svpwm);
#endif

#if (APP_DWT_TIMING_ENABLE != 0u)
  AppFocTiming_Update(AppFocTiming_GetCycle() - hf_start_cycles);
#endif
}

FOC_StateTypeDef AppFoc_GetState(void)
{
  return s_state;
}

uint8_t AppFoc_GetTraceSnapshot(FOC_TraceSnapshot_t *snapshot)
{
#if (FOC_HF_DEBUG_SNAPSHOT_ENABLE != 0u)
  return FOC_Trace_GetSnapshot(&s_trace, snapshot);
#else
  (void)snapshot;
  return 0u;
#endif
}

uint8_t AppFoc_GetRuntimeSnapshot(FOC_RuntimeSnapshot_t *snapshot)
{
  uint32_t seq_start;
  uint32_t seq_end;
  uint8_t retry = 8u;

  if (snapshot == 0)
  {
    return 0u;
  }

  do
  {
    seq_start = s_runtime_seq;
    if ((seq_start & 1u) != 0u)
    {
      continue;
    }

    *snapshot = s_runtime_snapshot;
    seq_end = s_runtime_seq;

    if ((seq_start == seq_end) && ((seq_end & 1u) == 0u))
    {
      return 1u;
    }
  } while (--retry != 0u);

  return 0u;
}

uint8_t AppFoc_GetHfTimingSnapshot(FOC_HfTimingSnapshot_t *snapshot)
{
  return AppFocTiming_GetSnapshot(snapshot);
}

void AppFoc_ResetHfTimingStats(void)
{
  AppFocTiming_Reset();
}

uint8_t AppFoc_GetFaultSnapshot(FOC_FaultSnapshot_t *snapshot)
{
  if ((snapshot == 0) || (s_fault_snapshot.valid == 0u))
  {
    return 0u;
  }

  *snapshot = s_fault_snapshot;
  return 1u;
}

void AppFoc_ClearFaultSnapshot(void)
{
  s_fault_snapshot.valid = 0u;
}

void AppFoc_SetSpeedTargetMechHz(float mech_hz)
{
  float target_rad_s = app_foc_mech_hz_to_rad_s(mech_hz);

  if (target_rad_s < 0.0f)
  {
    target_rad_s = 0.0f;
  }
  else if (target_rad_s > FOC_SPEED_TARGET_MAX_MECH_RAD_S)
  {
    target_rad_s = FOC_SPEED_TARGET_MAX_MECH_RAD_S;
  }

  /* 只修改最终目标，不直接改 speed_ref_cmd。
   * speed_ref_cmd 在中频速度环里按 FOC_SPEED_REF_RAMP_HZ_PER_S 斜坡追目标，
   * 避免按键/上位机改速时给速度环一个阶跃。 */
  FOC_SpeedLoopSetTargetMechRadS(&s_speed_loop, target_rad_s);
}

uint32_t AppFoc_GetFaultFlags(void)
{
  return s_fault_flags;
}
