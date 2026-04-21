#ifndef __FOC_CONFIG_H__
#define __FOC_CONFIG_H__
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

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
#define FOC_SVPWM_MIN_CCR            (20u)       /* 留一点边界余量，避免比较值贴死到 0。 */
#define FOC_SVPWM_MAX_CCR            (FOC_PWM_ARR - 20u) /* 留一点边界余量，避免比较值贴死到 ARR。 */

/* ==================== 电机参数 ==================== */

#define FOC_POLE_PAIRS               (4.0f)      /* 电机极对数，用于机械角速度和电角速度之间的换算。 */
#define FOC_RS_OHM                   (0.53f)     /* 定子电阻，单位欧姆，影响电压降补偿和磁链积分。 */
#define FOC_LS_H                     (0.0006f)   /* 定子电感，单位亨利，影响磁链中电感分量的扣除。 */
#define FOC_FLUX_WB                  (0.0078f)   /* 目标磁链幅值，单位韦伯，用于观测器非线性约束。 */
#define FOC_FLUX_LIMIT_RATIO         (1.50f)     /* 磁链只在超过目标幅值该倍数时限幅，避免每拍硬归一化。 */

/* ==================== 电流环参数 ==================== */

#define FOC_ID_REF_A                 (0.0f)      /* d 轴电流参考，第一阶段先不给励磁电流。 */
#define FOC_IQ_REF_A                 (1.0f)      /* q 轴电流参考，增加低速开环跟随转矩。 */
#define FOC_ID_KP                    (3.770f)    /* d 轴电流 PI 比例增益，约等于 FOC_LS_H * 2*pi*1000。 */
#define FOC_ID_KI                    (3330.0f)   /* d 轴电流 PI 积分增益，约等于 FOC_RS_OHM * 2*pi*1000。 */
#define FOC_IQ_KP                    (3.770f)    /* q 轴电流 PI 比例增益，第一阶段按 Ld=Lq=Ls 处理。 */
#define FOC_IQ_KI                    (3330.0f)   /* q 轴电流 PI 积分增益，第一阶段按 d/q 同参数处理。 */
#define FOC_CURR_LOOP_V_LIMIT        (6.0f)      /* 单个 PI 输出电压限幅。 */
#define FOC_DQ_VOLT_LIMIT            (6.0f)      /* dq 电压矢量总幅值限幅。 */
#define FOC_VD_CMD_SIGN              (-1.0f)     /* d 轴电压输出极性，实测 +Vd 得到 -Id，因此这里取反。 */
#define FOC_VQ_CMD_SIGN              (-1.0f)     /* q 轴电压输出极性，实测 +Vq 得到 -Iq，因此这里取反。 */
#define FOC_DQ_TEST_VD_V             (1.5f)      /* dq 电压注入测试的 d 轴电压，当前验证 d 轴极性。 */
#define FOC_DQ_TEST_VQ_V             (0.0f)      /* dq 电压注入测试的 q 轴电压。 */

/* ==================== 启动 / 对齐 / 开环参数 ==================== */

#define FOC_ALIGN_TIME_MS            (30000u)      /* 对齐阶段持续时间，单位 ms，过短可能导致转子未吸稳就进入开环。 */
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

#define FOC_OBS_ENABLE_FREQ_HZ       (8.0f)      /* 机械频率太低时，不对 observer 做锁定判定。 */
#define FOC_OBS_LOCK_ERR_DEG         (25.0f)     /* 角差在这个范围内，才认为有机会锁住。 */
#define FOC_OBS_LOCK_HOLD_MS         (200u)      /* 条件持续这么久，才把状态置为 locked。 */
#define FOC_OBS_THETA_COMP_RAD       (-0.4177f)   /* observer 原始角度补偿。 */
#define FOC_OBS_PLL_KP               (180.0f)    /* PLL 比例增益，调小可降低速度尖峰。 */
#define FOC_OBS_PLL_KI               (6000.0f)   /* PLL 积分增益，调小可降低低速抖动。 */
#define FOC_OBS_PLL_INTEGRAL_LIMIT   (700.0f)    /* PLL 积分项限幅，单位 rad/s。 */
#define FOC_OBS_SPEED_LIMIT_RAD_S    (800.0f)    /* observer 电角速度输出限幅。 */
#define FOC_OBS_SPEED_FILTER_ALPHA   (0.20f)     /* observer 速度一阶滤波系数，0~1，越小越平滑。 */

/* ==================== switchover 参数 ==================== */

#define FOC_SWITCHOVER_ENABLE                   (0u)    /* 1=允许 observer 接管控制角，0=只后台观察。 */
#define FOC_SWITCHOVER_MIN_MECH_FREQ_HZ         (6.0f)  /* 机械频率达到该值后，才允许进入接管判定。 */
#define FOC_SWITCHOVER_MAX_ANGLE_ERR_DEG        (8.0f)  /* 开环角和 observer 角误差小于该值，才认为角度可信。 */
#define FOC_SWITCHOVER_MAX_SPEED_ERR_RATIO      (0.50f) /* observer 电角速度与开环电角速度允许的相对误差。 */
#define FOC_SWITCHOVER_MAX_SPEED_ERR_MIN_RAD_S  (20.0f) /* 速度误差下限门限：低速时也保留一个最小绝对误差窗口。 */
#define FOC_SWITCHOVER_HOLD_MS                  (100u)  /* 接管条件连续满足这么久，才进入混合阶段。 */
#define FOC_SWITCHOVER_BLEND_MS                 (300u)  /* 从开环角平滑过渡到 observer 角所用时间。 */

/* ==================== 保护参数 ==================== */

#define FOC_CURRENT_LIMIT_A          (15.0f)     /* 单相电流限值，触发后直接进入故障态。 */
#define FOC_VBUS_UNDERVOLTAGE_V      (10.0f)     /* 母线太低时，SVPWM 已经不再可信。 */
#define FOC_VBUS_OVERVOLTAGE_V       (30.0f)     /* 母线太高时，先保护再说。 */

/* ==================== 调试参数 ==================== */

#define FOC_DEBUG_PRINT_DETAIL       (0u)        /* 0=精简打印，1=打印采样与电流环完整调试量。 */
#ifdef __cplusplus
}

#endif
#endif /* __FOC_CONFIG_H__ */
