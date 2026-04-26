#ifndef __FOC_TYPES_H__
#define __FOC_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * FOC 公共类型定义。
 *
 * 这里放的是跨模块共享的数据结构，主要给以下层次共用：
 * - App 层：主控编排与调试输出；
 * - Control 层：控制算法模块；
 * - BSP 层：采样、PWM 等底层支持。
 *
 * 原则：
 * 1. 这里只放“公共类型”，不放具体算法实现；
 * 2. 调试快照和运行快照统一定义在这里；
 * 3. 尽量避免到处声明零散 getter，改为结构化快照读取。
 *
 * 维护提醒：
 * 如果新增调试字段，优先考虑是否属于“高频底层调试”
 * 还是“应用层运行总览”，分别放到 FOC_DebugSnapshot_t
 * 或 FOC_RuntimeSnapshot_t 中，避免一个结构体无限膨胀。
 */

/*
 * FOC 运行状态。
 *
 * 说明：
 * - IDLE         : 空闲状态，PWM 通常保持安全中点；
 * - OFFSET_CALIB : 电流采样零点校准；
 * - ALIGN        : 转子对齐；
 * - OPENLOOP     : 开环运行；
 * - RUN          : 运行态（当前先保留为电流环/observer 接管后的执行态，便于后续挂速度环）；
 * - FAULT        : 故障锁定。
 */
typedef enum
{
  FOC_STATE_IDLE = 0,
  FOC_STATE_OFFSET_CALIB,
  FOC_STATE_ALIGN,
  FOC_STATE_OPENLOOP,
  FOC_STATE_RUN,
  FOC_STATE_FAULT
} FOC_StateTypeDef;

/*
 * 高频调试快照。
 *
 * 这个结构体更偏底层：
 * - PWM 比较值；
 * - 采样窗口；
 * - 原始 ADC 值；
 * - 三相电流；
 * 适合排查“采样时序 / 扇区 / 重构”这类问题。
 */
typedef struct
{
  uint32_t seq;   /* 快照序号。若使用双写保护，稳定快照通常为偶数。 */
  uint8_t sec;    /* 当前采样模块使用的扇区。 */
  uint8_t pwm_sec;/* 当前 PWM 输出扇区。 */
  uint8_t pair;   /* 当前可信两相组合：1=VW，2=UW，3=UV。 */
  uint8_t fb;     /* 采样点是否走回退策略。 */
  uint8_t hold;   /* 1=本拍采样窗口不足，电流反馈沿用上一拍有效值。 */
  uint8_t dyn;    /* 1=ADC触发点由动态窗口算法偏移。 */
  uint16_t ccru;  /* U 相 PWM 比较值。 */
  uint16_t ccrv;  /* V 相 PWM 比较值。 */
  uint16_t ccrw;  /* W 相 PWM 比较值。 */
  float t1;       /* 当前扇区第一个有效矢量作用时间。 */
  float t2;       /* 当前扇区第二个有效矢量作用时间。 */
  float t0;       /* 当前零矢量总作用时间。 */
  uint16_t win1;  /* 第一个有效采样窗口宽度。 */
  uint16_t win2;  /* 第二个有效采样窗口宽度。 */
  uint16_t smp;   /* 注入 ADC 触发点，通常对应 CCR4。 */
  uint16_t rawu;  /* U 相原始 ADC 采样值。 */
  uint16_t rawv;  /* V 相原始 ADC 采样值。 */
  uint16_t raww;  /* W 相原始 ADC 采样值。 */
  float iu;       /* U 相电流，单位 A。 */
  float iv;       /* V 相电流，单位 A。 */
  float iw;       /* W 相电流，单位 A。 */
  float isum;     /* 三相电流和，理想情况下应接近 0。 */
} FOC_DebugSnapshot_t;

/* 开环阶段快照。 */
typedef struct
{
  float freq_hz;      /* 当前开环机械频率，单位 Hz。 */
  float theta_e_rad;  /* 当前开环电角度，单位 rad。 */
} FOC_OpenLoopSnapshot_t;

/* 观测器快照。 */
typedef struct
{
  float flux_theta_rad;  /* 磁链角。 */
  float theta_rad;       /* 观测器估算角度。 */
  float speed_rad_s;     /* 观测器估算角速度。 */
  float angle_err_deg;   /* 观测器相位误差，单位 deg。 */
  float valpha_v;        /* 输入到观测器的 alpha 轴电压。 */
  float vbeta_v;         /* 输入到观测器的 beta 轴电压。 */
  float ialpha_a;        /* 输入到观测器的 alpha 轴电流。 */
  float ibeta_a;         /* 输入到观测器的 beta 轴电流。 */
  float pll_err;         /* PLL 归一化相位误差。 */
  float pll_integral;    /* PLL 积分项，单位 rad/s。 */
  float pll_speed_raw_rad_s;  /* PLL 未滤波电角速度，单位 rad/s。 */
  float pll_speed_filt_rad_s; /* PLL 滤波后电角速度，单位 rad/s。 */
  float flux_alpha;      /* 转子磁链 alpha 分量。 */
  float flux_beta;       /* 转子磁链 beta 分量。 */
  float flux_mag;        /* 转子磁链幅值。 */
  float vbus_v;          /* 当前母线电压。 */
  uint8_t locked;        /* 观测器是否锁定。 */
  uint8_t speed_limit_hit;    /* PLL 速度限幅是否接近打满。 */
  uint8_t integral_limit_hit; /* PLL 积分限幅是否接近打满。 */
} FOC_ObserverSnapshot_t;

/* 采样模块快照。 */
typedef struct
{
  float vbus_v;          /* 母线电压。 */
  float iu_a;            /* U 相电流。 */
  float iv_a;            /* V 相电流。 */
  float iw_a;            /* W 相电流。 */
  float iu_offset_v;     /* U 相电流采样零偏电压。 */
  float iv_offset_v;     /* V 相电流采样零偏电压。 */
  float iw_offset_v;     /* W 相电流采样零偏电压。 */
  uint16_t raw_u;        /* U 相重构前原始 ADC 值。 */
  uint16_t raw_v;        /* V 相重构前原始 ADC 值。 */
  uint16_t raw_w;        /* W 相重构前原始 ADC 值。 */
  uint16_t adc1_j1_raw;  /* ADC1 注入通道 1 原始值。 */
  uint16_t adc1_j2_raw;  /* ADC1 注入通道 2 原始值。 */
  uint16_t adc2_j1_raw;  /* ADC2 注入通道 1 原始值。 */
  uint16_t adc2_j2_raw;  /* ADC2 注入通道 2 原始值。 */
  uint16_t adc2_j3_raw;  /* ADC2 注入通道 3 原始值。 */
  uint8_t trusted_pair;  /* 当前可信两相组合。 */
  uint8_t fallback_request;      /* 1=本拍 PWM 采样窗口不足，请求 fallback。 */
  uint8_t used_hold_last_current;/* 1=本拍实际沿用上一拍有效电流。 */
  uint16_t fallback_hold_count;  /* 连续 hold-last 计数，排查低占空比采样窗口问题。 */
} FOC_SamplingSnapshot_t;

/* 电流环快照。 */
typedef struct
{
  float id_ref_a;         /* d 轴电流参考值。 */
  float iq_ref_a;         /* q 轴电流参考值。 */
  float id_meas_a;        /* d 轴电流反馈值。 */
  float iq_meas_a;        /* q 轴电流反馈值。 */
  float vd_pi_v;          /* d 轴 PI 原始输出，软接管时用于观察突变风险。 */
  float vd_cmd_v;         /* d 轴电压指令。 */
  float vq_cmd_v;         /* q 轴电压指令。 */
  float valpha_cmd_v;     /* alpha 轴电压指令。 */
  float vbeta_cmd_v;      /* beta 轴电压指令。 */
  float theta_ctrl_rad;   /* 本拍控制角。 */
  float d_axis_enable_k;   /* d 轴 PI 软接管系数，0=Iq-only，1=完整 d 轴接管。 */
  uint8_t run_iq_only;     /* 1=RUN 阶段当前使用 Iq-only 控制。 */
  uint8_t dt_enable;       /* 1=本拍电压输出启用了死区补偿。 */
} FOC_CurrentLoopSnapshot_t;

/* 速度环快照。 */
typedef struct
{
  float speed_ref_mech_rad_s;  /* 当前机械速度给定，单位 rad/s。 */
  float speed_target_mech_rad_s; /* 当前机械速度目标，单位 rad/s。 */
  float speed_fdb_mech_rad_s;  /* 当前机械速度反馈，单位 rad/s。 */
  float iq_ref_cmd_a;          /* 速度环输出的 q 轴电流给定。 */
  uint8_t enabled;             /* 速度环是否已经真正使能。 */
} FOC_SpeedLoopSnapshot_t;

/* 开环到观测器切换快照。 */
typedef struct
{
  uint8_t state;            /* 切换状态机当前状态。 */
  uint8_t observer_ready;   /* 观测器是否达到可接管条件。 */
  uint8_t ready_now;        /* 当前这一拍是否满足接管条件。 */
  uint32_t hold_ticks;      /* 连续满足接管条件的保持计数。 */
  uint32_t blend_ticks;     /* 混合接管已持续 tick 数。 */
  float blend_k;            /* 开环角和观测角的混合系数。 */
  float theta_open_rad;     /* 开环角。 */
  float theta_obs_rad;      /* 观测器角。 */
  float theta_ctrl_rad;     /* 实际送入控制器的角。 */
  float angle_err_deg;      /* 开环角与观测角误差。 */
  float open_speed_rad_s;   /* 开环电角速度。 */
  float obs_speed_rad_s;    /* 观测器估算角速度。 */
  float speed_err_rad_s;    /* 开环/观测器速度误差绝对值。 */
  uint8_t fail_reason;      /* 本拍接管条件失败原因位图：bit0=频率，bit1=角度，bit2=速度。 */
  uint8_t last_blend_reset_reason; /* 最近一次 BLEND 回退原因位图。 */
  uint32_t blend_reset_count;      /* BLEND 回退累计次数。 */
  float last_blend_reset_angle_err_deg; /* 最近一次 BLEND 回退瞬间的角度误差。 */
} FOC_SwitchoverSnapshot_t;

/*
 * 应用层运行总快照。
 *
 * 这个结构体面向“整体观察”：
 * 读取一次，就能同时拿到开环、观测器、采样、电流环、切换器
 * 的核心运行数据。
 */
typedef struct
{
  FOC_StateTypeDef state;                    /* 当前 FOC 主状态。 */
  uint32_t fault_flags;                       /* 当前/最近一次故障位图。 */
  uint8_t svpwm_sector;                      /* 当前 PWM 扇区。 */
  FOC_OpenLoopSnapshot_t openloop;          /* 开环状态。 */
  FOC_ObserverSnapshot_t observer;          /* 观测器状态。 */
  FOC_SamplingSnapshot_t sampling;          /* 采样状态。 */
  FOC_CurrentLoopSnapshot_t current_loop;   /* 电流环状态。 */
  FOC_SpeedLoopSnapshot_t speed_loop;       /* 速度环状态。 */
  FOC_SwitchoverSnapshot_t switchover;      /* 切换状态。 */
} FOC_RuntimeSnapshot_t;

/* 兼容旧命名。 */
typedef FOC_StateTypeDef FocState_t;
typedef FOC_DebugSnapshot_t FocDebugSnapshot_t;

#ifdef __cplusplus
}
#endif

#endif
