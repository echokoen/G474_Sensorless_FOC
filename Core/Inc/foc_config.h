#ifndef __FOC_CONFIG_H__
#define __FOC_CONFIG_H__
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

/* FOC 全局配置文件。
 *
 * 这个文件集中放所有“会影响控制行为”的工程参数：
 * - 硬件采样比例、PWM 周期、ADC 换算；
 * - 电机参数 Rs/Ls/磁链/极对数；
 * - 电流环、速度环、observer、switchover 的调试参数；
 * - 保护阈值和调试打印开关。
 *
 * 调参原则：
 * 1. 先确认硬件参数正确，再调控制器参数；
 * 2. 改宏值等价于改控制行为，必须带着日志验证；
 * 3. 采样、SVPWM、observer 的问题不要混在一起调。
 */

/* ==================== 硬件与采样参数 ==================== */

#define FOC_TS_SEC                   (6.25e-5f)  /* 控制周期，单位秒，必须与实际中断周期一致。 */
#define FOC_VBUS_RATIO               (25.0f)     /* 母线电压分压比例，用于把 ADC 值换算成实际母线电压。 */
#define FOC_SHUNT_OHM                (0.02f)     /* 电流采样电阻，单位欧姆。 */
#define FOC_GAIN                     (6.0f)      /* 电流采样放大倍数。 */
#define FOC_CURR_SIGN_U              (1.0f)      /* U 相电流方向符号，用于适配硬件接线方向。 */
#define FOC_CURR_SIGN_V              (1.0f)      /* V 相电流方向符号，用于适配硬件接线方向。 */
#define FOC_CURR_SIGN_W              (1.0f)      /* W 相电流方向符号，用于适配硬件接线方向。 */
#define FOC_PWM_ARR                  (5311u)     /* TIM1 自动重装载值，决定 PWM 分辨率和周期。 */
#define FOC_PWM_HALF_ARR             (2655u)     /* PWM 中点比较值，等效于零矢量输出。 */
#define FOC_ADC_FULL_SCALE           (4096.0f)   /* 12 位 ADC 换算分母，按正点原子例程使用 4096。 */
#define FOC_ADC_VREF                 (3.3f)      /* ADC 基准电压，单位伏。 */
#define FOC_CURRENT_OFFSET_CALIB_SAMPLES (4096u) /* 电流偏置校准采样数，样本越多越稳但启动越慢。 */
#define FOC_ADC_TRIG_MARGIN_CCR      (140u)      /* 动态采样窗口两侧预留边界，第一轮仍保留原宏名与入口。 */
#define FOC_ADC_TRIG_MIN_WINDOW_CCR  (320u)      /* 动态采样最小窗口阈值，当前主线仍维持固定中点采样。 */
#define FOC_ADC_TRIG_FALLBACK_CCR    (FOC_PWM_HALF_ARR) /* 没有足够宽窗口时回退到保守采样点。 */
#define FOC_ADC_TRIG_TEST_OFFSET_CCR (0)         /* 采样点 A/B 实验偏移量；0=中点，负值=提前，正值=滞后。 */
#define FOC_ADC_DYNAMIC_SAMPLE_ENABLE (1u)       /* 1=按可信两相低边公共导通窗口动态移动 ADC 触发点。 */
#define FOC_ADC_HOLD_LAST_ON_FALLBACK (1u)       /* 1=OPENLOOP/RUN 中窗口不足时沿用上一拍有效电流。 */
#define FOC_ADC_SAMPLE_MIN_CCR        (FOC_ADC_TRIG_MARGIN_CCR) /* ADC触发点允许的最小CCR，避开计数边界。 */
#define FOC_ADC_SAMPLE_MAX_CCR        (FOC_PWM_ARR - FOC_ADC_TRIG_MARGIN_CCR) /* ADC触发点允许的最大CCR。 */
#define FOC_SVPWM_MIN_CCR            (20u)       /* 留一点边界余量，避免比较值贴死到 0。 */
#define FOC_SVPWM_MAX_CCR            (FOC_PWM_ARR - 20u) /* 留一点边界余量，避免比较值贴死到 ARR。 */

/* ==================== 死区补偿参数 ==================== */
/* 死区补偿用于修正功率管上下桥互锁时间带来的相电压误差。
 * 当前实现按电流方向补偿，并在过零附近做平滑，避免符号抖动。
 */
#define FOC_DT_COMP_ENABLE            (1u)      /* 死区补偿总开关，1=启用，0=关闭。 */
#define FOC_DT_COMP_ENABLE_ALIGN      (0u)      /* ALIGN阶段死区补偿开关，当前隔离实验临时关闭。 */
#define FOC_DT_COMP_ENABLE_OPENLOOP   (0u)      /* OPENLOOP阶段死区补偿开关。 */
#define FOC_DT_COMP_ENABLE_RUN        (0u)      /* RUN阶段死区补偿开关，当前隔离实验临时关闭。 */
#define FOC_DT_COMP_SIGN              (1.0f)    /* 死区补偿方向，若补偿后效果变差可改为-1.0f。 */
#define FOC_DT_COMP_CURR_TH_A         (0.30f)   /* 电流过零平滑阈值，单位A。 */
#define FOC_DT_COMP_K_BASE            (0.016f)  /* 死区补偿基准系数，按 K = Td / Ts 公式估算。 */
#define FOC_DT_COMP_K_MAX             (0.020f)  /* 死区补偿系数上限，防止补偿过大。 */
#define FOC_DT_COMP_MIN_VBUS_V        (8.0f)    /* 最小母线电压门限，低于该值时不进行补偿。 */

/* ==================== 电机参数 ==================== */
/* 这些参数会直接进入 observer、PI 参数估算和速度换算。
 * 如果 Rs/Ls/Flux/PolePairs 不准确，最明显的影响通常是：
 * - observer 角度/速度漂移；
 * - 低速锁相困难；
 * - 电流环带宽和实际响应不一致。
 */

#define FOC_POLE_PAIRS               (4.0f)      /* 电机极对数，用于机械角速度和电角速度之间的换算。 */
#define FOC_RS_OHM                   (0.53f)     /* 定子电阻，单位欧姆，影响电压降补偿和磁链积分。 */
#define FOC_LS_H                     (0.0006f)   /* 定子电感，单位亨利，影响磁链中电感分量的扣除。 */
#define FOC_FLUX_WB                  (0.0078f)   /* 目标磁链幅值，单位韦伯，用于观测器非线性约束。 */
#define FOC_FLUX_LIMIT_RATIO         (1.50f)     /* 磁链只在超过目标幅值该倍数时限幅，避免每拍硬归一化。 */

/* ==================== 电流环参数 ==================== */
/* 电流环是当前工程最核心的闭环。
 * 真实链路为：
 * id/iq ref -> PI -> 电压极性映射 -> dq 矢量限幅 -> InvPark -> SVPWM。
 */

#define FOC_ID_REF_A                 (0.0f)      /* d 轴电流参考，第一阶段先不给励磁电流。 */
#define FOC_IQ_REF_A                 (1.0f)      /* q 轴电流参考，增加低速开环跟随转矩。 */
#define FOC_IQ_KP                    (3.770f)    /* q 轴电流 PI 比例增益，第一阶段按 Ld=Lq=Ls 处理。 */
#define FOC_IQ_KI                    (3330.0f)   /* q 轴电流 PI 积分增益，第一阶段按 d/q 同参数处理。 */
#define FOC_ID_KP                    (FOC_IQ_KP * 0.5f)  /* d 轴 PI 先保守：比例为 q 轴一半。 */
#define FOC_ID_KI                    (FOC_IQ_KI * 0.2f)  /* d 轴 PI 先保守：积分为 q 轴 20%。 */
#define FOC_CURR_LOOP_V_LIMIT        (13.5f)      /* 单个 PI 输出电压限幅。 */
#define FOC_DQ_VOLT_LIMIT            (13.5f)      /* dq 电压矢量总幅值限幅。 */
#define FOC_VD_CMD_SIGN              (-1.0f)     /* d 轴电压输出极性，实测 +Vd 得到 -Id，因此这里取反。 */
#define FOC_VQ_CMD_SIGN              (-1.0f)     /* q 轴电压输出极性，实测 +Vq 得到 -Iq，因此这里取反。 */
#define FOC_DQ_TEST_VD_V             (1.5f)      /* dq 电压注入测试的 d 轴电压，当前验证 d 轴极性。 */
#define FOC_DQ_TEST_VQ_V             (0.0f)      /* dq 电压注入测试的 q 轴电压。 */

/* ==================== 启动 / 对齐 / 开环参数 ==================== */
/* 启动流程相关参数。
 * OFFSET_CALIB 完成后进入 ALIGN，对齐结束后进入 OPENLOOP，
 * OPENLOOP 中 observer 后台跟踪，满足条件后切入 RUN。
 */

#define FOC_ALIGN_TIME_MS            (300u)      /* 对齐阶段持续时间，单位 ms，过短可能导致转子未吸稳就进入开环。 */
#define FOC_ALIGN_ID_REF_A           (0.8f)      /* 对齐阶段 d 轴电流给定，固定角度下用小电流把转子吸到对齐位置。 */
#define FOC_ALIGN_THETA_OFFSET_RAD   (0.0f)      /* 对齐角补偿。当前 V/F observer 标定阶段先归零。 */
#define FOC_OPENLOOP_VDQ_V           (6.0f)      /* 开环旋转时的电压幅值，太小带不动，太大会过流。 */
#define FOC_OPENLOOP_START_FREQ_HZ   (0.2f)      /* 开环起步机械频率，低速起转时先保守一些。 */
#define FOC_OPENLOOP_TARGET_FREQ_HZ  (15.0f)      /* I/F 低速调试目标机械频率。 */
#define FOC_OPENLOOP_RAMP_TIME_MS    (3000.0f)   /* 低速测试阶段放慢爬坡，方便观察 id/iq 方向。 */

/* 开环阶段控制模式。 */

#define FOC_CTRL_MODE_OPENLOOP_VF      (0u)      /* 原有 V/F 小电压开环。 */
#define FOC_CTRL_MODE_OPENLOOP_CURRENT (1u)      /* 开环角度 + dq 电流闭环。 */
#define FOC_CTRL_MODE_OPENLOOP_DQ_VOLT_TEST (2u) /* 开环角度 + 固定 dq 电压注入，用于验证坐标方向。 */
#ifndef FOC_OPENLOOP_CTRL_MODE
#define FOC_OPENLOOP_CTRL_MODE         (FOC_CTRL_MODE_OPENLOOP_CURRENT) /* 回到开环角度 + dq 电流闭环，observer 仍只后台观察。 */
#endif

/* ==================== observer 参数 ==================== */
/* FluxObserver_PLL 相关参数。
 * KP/KI 主要影响 PLL 跟踪速度和速度估计噪声；
 * THETA_COMP 用于补偿观测器角与控制角之间的固定相位偏差。
 */

#define FOC_OBS_ENABLE_FREQ_HZ       (10.0f)      /* 机械频率太低时，不对 observer 做锁定判定。 */
#define FOC_OBS_LOCK_ERR_DEG         (25.0f)     /* 角差在这个范围内，才认为有机会锁住。 */
#define FOC_OBS_LOCK_HOLD_MS         (200u)      /* 条件持续这么久，才把状态置为 locked。 */
#define FOC_OBS_THETA_COMP_RAD       (2.2003f)    /* observer 角度补偿：当前 angle_err≈-36deg，补偿值减小 0.6283rad。 */
#define FOC_OBS_PLL_KP               (120.0f)    /* PLL 比例增益，调小可降低速度尖峰。 */
#define FOC_OBS_PLL_KI               (3000.0f)   /* PLL 积分增益，调小可降低低速抖动。 */
#define FOC_OBS_PLL_INTEGRAL_LIMIT   (1800.0f)    /* PLL 积分项限幅，单位 rad/s。 */
#define FOC_OBS_SPEED_LIMIT_RAD_S    (2200.0f)    /* observer 电角速度输出限幅。 */
#define FOC_OBS_SPEED_FILTER_ALPHA   (0.05f)     /* observer 速度一阶滤波系数，0~1，越小越平滑。 */

/* ==================== switchover 参数 ==================== */
/* 开环到 observer 接管参数。
 * 接管不是只看角度，还会同时看速度误差和连续保持时间，
 * 目的是避免 observer 瞬间“看起来对了”就贸然切入 RUN。
 */

#define FOC_SWITCHOVER_ENABLE                   (1u)    /* 1=允许 observer 接管控制角，0=只后台观察。 */
#define FOC_SWITCHOVER_MIN_MECH_FREQ_HZ         (10.0f)  /* 机械频率达到该值后，才允许进入接管判定。 */
#define FOC_SWITCHOVER_MAX_ANGLE_ERR_DEG        (15.0f) /* 开环角和 observer 角误差小于该值，才认为角度可信。 */
#define FOC_SWITCHOVER_MAX_SPEED_ERR_RATIO      (0.60f) /* observer 电角速度与开环电角速度允许的相对误差。 */
#define FOC_SWITCHOVER_MAX_SPEED_ERR_MIN_RAD_S  (20.0f) /* 速度误差下限门限：低速时也保留一个最小绝对误差窗口。 */
#define FOC_SWITCHOVER_HOLD_MS                  (300u)  /* 接管条件连续满足这么久，才进入混合阶段。 */
#define FOC_SWITCHOVER_BLEND_MS                 (800u)  /* 从开环角平滑过渡到 observer 角所用时间。 */
#define FOC_SWITCHOVER_BLEND_MAX_ANGLE_ERR_DEG  (35.0f) /* BLEND 中放宽角度门限，避免单拍角度毛刺打回。 */
#define FOC_SWITCHOVER_BLEND_FAIL_HOLD_TICKS    (20u)    /* BLEND 中连续失败这么多拍，才真正退回 OPENLOOP。 */

/* ==================== 速度环参数 ==================== */
/* 速度环当前是 RUN 阶段的外环，只输出 iq_ref。
 * 调试阶段先保守限幅，避免 observer 刚接管时速度环把 q 轴电流拉得过猛。
 */

#define FOC_SPEED_REF_MECH_HZ        (12.0f)     /* RUN 阶段默认机械速度目标，当前版本进入 RUN 时先不直接跳到该值。 */
#define FOC_SPEED_TARGET_MAX_MECH_RAD_S (290.0f) /* 速度目标上限，单位机械 rad/s。 */
#define FOC_SPEED_KP                 (0.12f)     /* 速度环比例增益；加快 RUN 后速度跟随。 */
#define FOC_SPEED_KI                 (4.0f)      /* 速度环积分增益；提高低于目标速度时的补扭矩速度。 */
#define FOC_SPEED_IQ_MIN_A           (0.5f)      /* RUN 后保留最小正扭矩底座，避免速度环把 iq_ref 拉得过低。 */
#define FOC_SPEED_IQ_MAX_A           (7.0f)      /* 速度环输出的 q 轴电流上限，短时放开到 7A 验证速度余量。 */
#define FOC_SPEED_ENABLE_DELAY_MS    (500u)      /* 进入 RUN 后先 observer-only hold 一小段，再真正开启速度环。 */
#define FOC_SPEED_REF_RAMP_HZ_PER_S  (3.0f)      /* 速度给定斜率；当前默认目标由接管瞬间的实际速度初始化。 */
#define FOC_SPEED_FDB_FILTER_ALPHA   (0.08f)     /* RUN 阶段速度反馈一阶低通系数。 */
#define FOC_SPEED_TAKEOVER_IQ_HOLD_A (0.30f)     /* 速度环刚接入后的最小正扭矩底座，避免 iq_ref 立即掉空。 */
#define FOC_SPEED_TAKEOVER_HOLD_MS   (300u)      /* 速度环刚接入后的底座保持时间。 */
#define FOC_SPEED_IQ_SLEW_A_PER_S    (5.0f)      /* 速度环 iq_ref 最大变化率，避免输出突变，同时保证加速响应。 */
#define FOC_SPEED_LOOP_ENABLE_IN_RUN (1u)        /* 隔离实验：1=RUN 阶段启用速度环，只让它输出 iq_ref。 */
#define FOC_D_AXIS_SOFT_RAMP_MS      (1000u)     /* OPENLOOP 开始后，d_axis_gain 从 0 爬升到 1 的时间。 */
#define FOC_D_AXIS_VD_LIMIT_START_V  (1.0f)      /* d 轴刚释放时的 vd 单独限幅。 */
#define FOC_D_AXIS_VD_LIMIT_END_V    (4.0f)      /* d 轴释放完成后的 vd 单独限幅，第一版不超过 ±4V。 */

/* ==================== 保护参数 ==================== */
/* 软件保护阈值。
 * 注意：软件保护依赖 ADC 采样结果，不能替代硬件过流保护。
 */

#define FOC_CURRENT_LIMIT_A          (15.0f)     /* 单相电流限值，触发后直接进入故障态。 */
#define FOC_VBUS_UNDERVOLTAGE_V      (10.0f)     /* 母线太低时，SVPWM 已经不再可信。 */
#define FOC_VBUS_OVERVOLTAGE_V       (30.0f)     /* 母线太高时，先保护再说。 */

/* ==================== 调试参数 ==================== */
/* 调试打印开关。
 * 打印过多会占用串口和 CPU，排查实时问题时要注意打印周期。
 */

#define FOC_DEBUG_PRINT_DETAIL       (0u)        /* 0=精简打印，1=打印采样与电流环完整调试量。 */
#ifdef __cplusplus
}

#endif
#endif /* __FOC_CONFIG_H__ */
