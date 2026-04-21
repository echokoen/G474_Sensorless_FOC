#include "app_foc.h"

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

#include "PMSM_Control_Core/FluxObserver_PLL.h"
#include "bsp_adc.h"
#include "bsp_gpio.h"
#include "bsp_tim.h"
#include "foc_current_loop.h"
#include "foc_config.h"
#include "foc_debug.h"
#include "foc_observer.h"
#include "foc_openloop.h"
#include "foc_protection.h"
#include "foc_pwm.h"
#include "foc_sampling.h"
#include "foc_switchover.h"

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
static FOC_SamplingData_t s_sampling = {0};
static FOC_PwmData_t s_pwm = {0};
static FOC_DebugData_t s_debug = {0};
static FOC_CurrentLoopData_t s_current_loop = {0};
static FOC_OpenLoopData_t s_openloop = {0};
static FOC_ObserverData_t s_observer = {0};
static FOC_SwitchoverData_t s_switchover = {0};
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
  APP_FOC_THETA_MODE_ALIGN = 0,
  APP_FOC_THETA_MODE_OPENLOOP,
  APP_FOC_THETA_MODE_OBSERVER
} AppFoc_ThetaMode_t;

typedef struct
{
  FOC_StateTypeDef state;
  AppFoc_ThetaMode_t theta_mode;
  float id_ref_a;
  float iq_ref_a;
  float speed_ref_mech_rad_s;
  uint8_t speed_loop_enable;
} AppFoc_Command_t;

typedef struct
{
  uint8_t offset_done;
  uint8_t observer_ready;
  uint32_t align_ticks;
  float mech_freq_hz;
  uint32_t fault_flags;
} AppFoc_Feedback_t;

static AppFoc_Command_t s_cmd = {0};
static AppFoc_Feedback_t s_feedback = {0};
static uint32_t s_last_medium_tick_ms = 0u;

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
static void app_foc_sampling_update(void)
{
  FOC_SamplingModule_SetSampleSector(&s_sampling, s_pwm.svpwm_sector);
  FOC_SamplingModule_Update(&s_sampling);
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
  FOC_PwmModule_SetTim1Mid(&s_pwm);
}

/*
 * 收口到故障状态。
 *
 * 该函数本身不做复杂判定，只调用保护模块统一执行：
 * - 关闭驱动；
 * - PWM 回中点；
 * - 状态锁到 FAULT。
 */
static void app_foc_stop_to_fault(void)
{
  if (s_fault_flags == FOC_FAULT_NONE)
  {
    /* 若外部主动停机走到这里，没有具体保护原因时保留一个兜底标记。 */
    s_fault_flags = 0x80000000u;
  }
  FOC_ProtectionApplyStop((FOC_StateTypeDef *)&s_state, &s_pwm);
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
  snapshot->svpwm_sector = s_pwm.svpwm_sector;

  /* 开环信息。 */
  snapshot->openloop.freq_hz = s_openloop.mech_freq_hz;
  snapshot->openloop.theta_e_rad = s_openloop.theta_e_rad;

  /* 观测器信息。 */
  snapshot->observer.flux_theta_rad = FOC_ObserverGetFluxThetaRad(&s_observer);
  snapshot->observer.theta_rad = s_observer.theta_rad;
  snapshot->observer.speed_rad_s = s_observer.speed_rad_s;
  snapshot->observer.angle_err_deg = s_observer.angle_err_deg;
  snapshot->observer.vbus_v = s_sampling.vbus_v;
  snapshot->observer.locked = s_observer.locked;
  FOC_ObserverGetInputs(&s_observer,
                        &snapshot->observer.valpha_v,
                        &snapshot->observer.vbeta_v,
                        &snapshot->observer.ialpha_a,
                        &snapshot->observer.ibeta_a);

  /* 采样信息。 */
  snapshot->sampling.vbus_v = s_sampling.vbus_v;
  snapshot->sampling.iu_a = s_sampling.iu_a;
  snapshot->sampling.iv_a = s_sampling.iv_a;
  snapshot->sampling.iw_a = s_sampling.iw_a;
  snapshot->sampling.iu_offset_v = s_sampling.iu_offset_v;
  snapshot->sampling.iv_offset_v = s_sampling.iv_offset_v;
  snapshot->sampling.iw_offset_v = s_sampling.iw_offset_v;
  snapshot->sampling.raw_u = s_sampling.raw_u;
  snapshot->sampling.raw_v = s_sampling.raw_v;
  snapshot->sampling.raw_w = s_sampling.raw_w;
  snapshot->sampling.adc1_j1_raw = s_sampling.adc1_j1_raw;
  snapshot->sampling.adc1_j2_raw = s_sampling.adc1_j2_raw;
  snapshot->sampling.adc2_j1_raw = s_sampling.adc2_j1_raw;
  snapshot->sampling.adc2_j2_raw = s_sampling.adc2_j2_raw;
  snapshot->sampling.adc2_j3_raw = s_sampling.adc2_j3_raw;
  snapshot->sampling.trusted_pair = s_sampling.trusted_current_pair;

  /* 电流环信息。 */
  snapshot->current_loop.id_ref_a = s_current_loop.id_ref_a;
  snapshot->current_loop.iq_ref_a = s_current_loop.iq_ref_a;
  snapshot->current_loop.id_meas_a = s_current_loop.id_meas_a;
  snapshot->current_loop.iq_meas_a = s_current_loop.iq_meas_a;
  snapshot->current_loop.vd_cmd_v = s_current_loop.vd_cmd_v;
  snapshot->current_loop.vq_cmd_v = s_current_loop.vq_cmd_v;
  snapshot->current_loop.valpha_cmd_v = s_current_loop.valpha_cmd_v;
  snapshot->current_loop.vbeta_cmd_v = s_current_loop.vbeta_cmd_v;
  snapshot->current_loop.theta_ctrl_rad = s_current_loop.theta_ctrl_rad;

  /* 开环到观测器切换信息。 */
  snapshot->switchover.state = (uint8_t)s_switchover.state;
  snapshot->switchover.observer_ready = s_switchover.observer_ready;
  snapshot->switchover.blend_k = s_switchover.blend_k;
  snapshot->switchover.theta_open_rad = s_openloop.theta_e_rad;
  snapshot->switchover.theta_obs_rad = s_observer.theta_rad;
  snapshot->switchover.theta_ctrl_rad = s_current_loop.theta_ctrl_rad;
  snapshot->switchover.angle_err_deg = s_observer.angle_err_deg;
  snapshot->switchover.open_speed_rad_s = FOC_OpenLoopGetElecSpeedRadPerSec(&s_openloop);
  snapshot->switchover.obs_speed_rad_s = s_observer.speed_rad_s;
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

  app_foc_force_mid_output();

  if (s_openloop.align_ticks == 0u)
  {
    app_foc_drive_enable(1u);
  }

  app_foc_sampling_update();
  s_fault_flags = FOC_ProtectionCheckFault(&s_sampling);
  if (s_fault_flags != FOC_FAULT_NONE)
  {
    app_foc_stop_to_fault();
    return;
  }

  /* 对齐阶段改为：
   * - 电角固定在 0；
   * - 通过 d 轴电流闭环建立静止磁场；
   * - q 轴给 0，避免对齐阶段引入额外转矩。
   *
   * 这样比固定电压对齐更柔和，也更符合当前主线已经切到电流环的结构。
   */
  s_openloop.theta_e_rad = 0.0f;
  FOC_CurrentLoopRunWithRef(&s_current_loop,
                            &s_sampling,
                            s_openloop.theta_e_rad,
                            FOC_ALIGN_ID_REF_A,
                            0.0f,
                            &valpha,
                            &vbeta);
  /* 对齐阶段只保留 d 轴磁场，直接把 q 轴电压清零，避免额外转矩导致持续抖动。 */
  vdq.d = s_current_loop.vd_cmd_v;
  vdq.q = 0.0f;
  ClarkePark_InvPark(&vdq, s_openloop.theta_e_rad, &vab);
  s_current_loop.vq_cmd_v = 0.0f;
  s_current_loop.valpha_cmd_v = vab.alpha;
  s_current_loop.vbeta_cmd_v = vab.beta;
  valpha = s_current_loop.valpha_cmd_v;
  vbeta = s_current_loop.vbeta_cmd_v;
  FOC_PwmModule_ApplySvpwm(&s_pwm, valpha, vbeta, s_sampling.vbus_v);

  s_openloop.align_ticks++;
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
  ClarkePark_AlphaBeta_t vab = {0.0f, 0.0f};
  ClarkePark_DQ_t vdq = {0.0f, 0.0f};
  float theta_ctrl;
  float valpha = 0.0f;
  float vbeta = 0.0f;

  app_foc_sampling_update();
  s_fault_flags = FOC_ProtectionCheckFault(&s_sampling);
  if (s_fault_flags != FOC_FAULT_NONE)
  {
    app_foc_stop_to_fault();
    return;
  }

  FOC_OpenLoopAdvance(&s_openloop);
  FOC_SwitchoverUpdate(&s_switchover, &s_openloop, &s_observer);
  theta_ctrl = FOC_SwitchoverSelectTheta(&s_switchover, &s_openloop, &s_observer);

#if (FOC_OPENLOOP_CTRL_MODE == FOC_CTRL_MODE_OPENLOOP_CURRENT)
  FOC_CurrentLoopRunOpenLoopCurrent(&s_current_loop, &s_sampling, theta_ctrl, &valpha, &vbeta);
  /* 当前调试版本先把开环阶段简化为“只保留 q 轴转矩推进”：
   * - ALIGN 阶段只保留 Vd；
   * - ALIGN 之后把 Vd_cmd 直接清零；
   * 这样可以把 d 轴补偿从开环链路里拿掉，先把 q 轴推进方向验证干净。 */
  vdq.d = 0.0f;
  vdq.q = s_current_loop.vq_cmd_v;
  ClarkePark_InvPark(&vdq, theta_ctrl, &vab);
  s_current_loop.vd_cmd_v = 0.0f;
  s_current_loop.valpha_cmd_v = vab.alpha;
  s_current_loop.vbeta_cmd_v = vab.beta;
  valpha = s_current_loop.valpha_cmd_v;
  vbeta = s_current_loop.vbeta_cmd_v;
#elif (FOC_OPENLOOP_CTRL_MODE == FOC_CTRL_MODE_OPENLOOP_DQ_VOLT_TEST)
  FOC_CurrentLoopRunOpenLoopVoltTest(&s_current_loop, &s_sampling, theta_ctrl, &valpha, &vbeta);
#else
  FOC_CurrentLoopRunOpenLoopVf(&s_current_loop, &s_sampling, theta_ctrl, &valpha, &vbeta);
#endif

  FOC_PwmModule_ApplySvpwm(&s_pwm, valpha, vbeta, s_sampling.vbus_v);

  FOC_ObserverUpdate(&s_observer,
                     &s_sampling,
                     valpha,
                     vbeta,
                     s_openloop.theta_e_rad,
                     s_openloop.mech_freq_hz);
}

static void app_foc_sync_feedback_from_modules(void)
{
  s_feedback.offset_done = (s_sampling.offset_samples >= FOC_CURRENT_OFFSET_CALIB_SAMPLES) ? 1u : 0u;
  s_feedback.observer_ready = s_switchover.observer_ready;
  s_feedback.align_ticks = s_openloop.align_ticks;
  s_feedback.mech_freq_hz = s_openloop.mech_freq_hz;
  s_feedback.fault_flags = s_fault_flags;
}

static void app_foc_medium_reset_command_defaults(void)
{
  s_cmd.state = s_state;
  s_cmd.theta_mode = APP_FOC_THETA_MODE_OPENLOOP;
  s_cmd.id_ref_a = FOC_ID_REF_A;
  s_cmd.iq_ref_a = FOC_IQ_REF_A;
  s_cmd.speed_ref_mech_rad_s = 0.0f;
  s_cmd.speed_loop_enable = 0u;
}

static void app_foc_medium_handle_state_machine(void)
{
  static FOC_StateTypeDef s_prev_state = FOC_STATE_IDLE;
  const uint32_t align_ticks_target =
      (uint32_t)((FOC_ALIGN_TIME_MS * 0.001f) / FOC_TS_SEC);
  const uint8_t state_changed = (s_state != s_prev_state) ? 1u : 0u;

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
        FOC_SwitchoverReset(&s_switchover);
        FOC_CurrentLoopReset(&s_current_loop);
        FOC_OpenLoopResetAlign(&s_openloop);
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
        FOC_SwitchoverReset(&s_switchover);
        FOC_CurrentLoopReset(&s_current_loop);
        FOC_OpenLoopResetAlign(&s_openloop);
      }

      if (s_feedback.offset_done != 0u)
      {
        FOC_OpenLoopResetAlign(&s_openloop);
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
        FOC_OpenLoopResetAlign(&s_openloop);
      }

      if (s_feedback.align_ticks >= align_ticks_target)
      {
        /* 对齐结束后，把开环、电流环、接管器清一次，
         * 保证进入 OPENLOOP 时从干净状态开始。 */
        FOC_OpenLoopResetRun(&s_openloop);
        FOC_SwitchoverReset(&s_switchover);
        FOC_CurrentLoopReset(&s_current_loop);
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
        FOC_SwitchoverReset(&s_switchover);
        FOC_CurrentLoopReset(&s_current_loop);
        FOC_OpenLoopResetRun(&s_openloop);
      }

      if (s_switchover.state == FOC_SW_STATE_OBSERVER)
      {
        s_state = FOC_STATE_RUN;
      }
      break;

    case FOC_STATE_RUN:
      /* 运行阶段：
       * 当前版本先只用 observer 角度运行；
       * 速度环框架预留，但先不真正启用。 */
      s_cmd.theta_mode = APP_FOC_THETA_MODE_OBSERVER;
      s_cmd.speed_loop_enable = 0u;

      if (state_changed != 0u)
      {
        /* 刚进入 RUN 时清一次电流环，避免上一阶段积分残留。 */
        FOC_CurrentLoopReset(&s_current_loop);
      }

      /* 最小版本里，如果 observer 丢失，先退回 OPENLOOP。
       * 后续再细化成 RUN -> BLEND -> OPENLOOP 的分级回退。 */
      if (s_switchover.state != FOC_SW_STATE_OBSERVER)
      {
        FOC_OpenLoopResetRun(&s_openloop);
        FOC_SwitchoverReset(&s_switchover);
        s_state = FOC_STATE_OPENLOOP;
      }
      break;

    case FOC_STATE_FAULT:
      /* 故障阶段：
       * 这里只保持安全命令；
       * 真正的停机收口由高频里的保护逻辑统一执行。 */
      s_cmd.theta_mode = APP_FOC_THETA_MODE_OPENLOOP;
      s_cmd.speed_loop_enable = 0u;
      s_cmd.id_ref_a = 0.0f;
      s_cmd.iq_ref_a = 0.0f;
      s_cmd.speed_ref_mech_rad_s = 0.0f;

      if (state_changed != 0u)
      {
        FOC_CurrentLoopReset(&s_current_loop);
        FOC_SwitchoverReset(&s_switchover);
      }
      break;

    default:
      s_state = FOC_STATE_IDLE;
      break;
  }

  s_cmd.state = s_state;
  s_prev_state = s_state;
}

static void app_foc_run_run(void)
{
  /* 运行态当前先复用开环阶段的实时控制链：
   * - 高频仍然只负责采样、observer、选角度、电流环与 SVPWM；
   * - 中频负责决定“什么时候进入 RUN”；
   * - 后续挂速度环时，只需在 RUN 状态里打开 speed_loop_enable 并更新给定。 */
  app_foc_run_openloop();
}

void AppFoc_Init(void)
{
  /* 初始化所有控制对象，并回到一个安全静止状态。 */
  s_state = FOC_STATE_IDLE;

  FOC_SamplingModule_Init(&s_sampling);
  FOC_PwmModule_Init(&s_pwm);
  FOC_DebugModule_Init(&s_debug);
  FOC_CurrentLoopInit(&s_current_loop);
  FOC_OpenLoopInit(&s_openloop);
  FOC_ObserverInit(&s_observer);
  FOC_SwitchoverInit(&s_switchover);
  app_foc_medium_reset_command_defaults();
  s_feedback.offset_done = 0u;
  s_feedback.observer_ready = 0u;
  s_feedback.align_ticks = 0u;
  s_feedback.mech_freq_hz = 0.0f;
  s_feedback.fault_flags = 0u;
  s_fault_flags = FOC_FAULT_NONE;
  s_last_medium_tick_ms = 0u;

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

  /* 启动 TIM1 更新中断，用于高频同步工作。 */
  HAL_TIM_Base_Start_IT(&htim1);

  /* 启动 ADC 常规/注入转换。 */
  HAL_ADC_Start(&hadc1);
  FOC_SamplingModule_SetAdc1Ready(&s_sampling, 1u);
  HAL_ADCEx_InjectedStart(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc2);

  /* 每次启动都要把内部状态重新清空。 */
  FOC_ObserverReset(&s_observer);
  FOC_SwitchoverReset(&s_switchover);
  FOC_CurrentLoopReset(&s_current_loop);
  FOC_OpenLoopResetAlign(&s_openloop);
  FOC_OpenLoopResetRun(&s_openloop);

  s_sampling.offset_samples = 0u;
  s_sampling.offset_sum_u = 0.0f;
  s_sampling.offset_sum_v = 0.0f;
  s_sampling.offset_sum_w = 0.0f;

  /* 启动后先保持中点输出并关闭驱动，等待零点校准。 */
  app_foc_medium_reset_command_defaults();
  s_feedback.offset_done = 0u;
  s_feedback.observer_ready = 0u;
  s_feedback.align_ticks = 0u;
  s_feedback.mech_freq_hz = 0.0f;
  s_feedback.fault_flags = 0u;
  s_fault_flags = FOC_FAULT_NONE;
  s_last_medium_tick_ms = HAL_GetTick();
  app_foc_force_mid_output();
  app_foc_drive_enable(0u);
  s_state = FOC_STATE_OFFSET_CALIB;
}

void AppFoc_Stop(void)
{
  /* 先走统一停机动作，再停止外设。 */
  app_foc_stop_to_fault();

  HAL_ADCEx_InjectedStop(&hadc1);
  HAL_ADCEx_InjectedStop_IT(&hadc2);
  HAL_ADC_Stop(&hadc1);

  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

  FOC_SamplingModule_SetAdc1Ready(&s_sampling, 0u);
  s_state = FOC_STATE_IDLE;
  s_fault_flags = FOC_FAULT_NONE;
  app_foc_medium_reset_command_defaults();
}

void AppFoc_MediumFreqTask(void)
{
  const uint32_t now_ms = HAL_GetTick();

  if ((now_ms - s_last_medium_tick_ms) < APP_FOC_MEDIUM_TASK_PERIOD_MS)
  {
    return;
  }

  s_last_medium_tick_ms = now_ms;
  app_foc_medium_handle_state_machine();
}

void AppFoc_HighFreqTask(void)
{
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
      app_foc_drive_enable(0u);
      FOC_SamplingModule_RunOffsetCalib(&s_sampling, &next_state);

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
      app_foc_stop_to_fault();
      break;

    case FOC_STATE_IDLE:
    default:
      app_foc_force_mid_output();
      break;
  }

  /* 每拍结束后刷新调试快照，供低频任务异步读取。 */
  FOC_DebugModule_UpdateSnapshot(&s_debug, &s_sampling, &s_pwm);
}

FOC_StateTypeDef AppFoc_GetState(void)
{
  return s_state;
}

uint8_t AppFoc_GetDebugSnapshot(FOC_DebugSnapshot_t *snapshot)
{
  return FOC_DebugModule_GetSnapshot(&s_debug, snapshot);
}

uint8_t AppFoc_GetRuntimeSnapshot(FOC_RuntimeSnapshot_t *snapshot)
{
  app_foc_fill_runtime_snapshot(snapshot);
  return (snapshot != 0) ? 1u : 0u;
}

uint32_t AppFoc_GetFaultFlags(void)
{
  return s_fault_flags;
}
