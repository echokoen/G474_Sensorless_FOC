#ifndef __FOC_CONFIG_H__
#define __FOC_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* FOC 基础配置区。
 *
 * 这里放的是“整个控制链都默认成立的前提”。
 * 一旦这些值和硬件不一致，observer、开环、保护逻辑都会一起偏。
 *
 * 因此调试时通常优先检查：
 * - 电机铭牌或实测参数是否与这里一致；
 * - 电流采样板的电阻和放大倍数是否一致；
 * - 控制周期是否真的与 `FOC_TS_SEC` 相符；
 * - 开环和保护阈值是不是过于激进或过于保守。
 */
/* 电机与控制基础参数 */
#define FOC_POLE_PAIRS               (4.0f)    /* 电机极对数，用于机械角速度和电角速度之间的换算。 */
#define FOC_RS_OHM                   (0.53f)   /* 定子电阻，单位欧姆，影响电压降补偿和磁链积分。 */
#define FOC_LS_H                     (0.0006f) /* 定子电感，单位亨利，影响磁链中电感分量的扣除。 */
#define FOC_FLUX_WB                  (0.0078f) /* 目标磁链幅值，单位韦伯，用于观测器非线性约束。 */
#define FOC_TS_SEC                   (6.25e-5f) /* 控制周期，单位秒，必须与实际中断周期一致。 */

/* 硬件换算参数 */
#define FOC_VBUS_RATIO               (25.0f)   /* 母线电压分压比例，用于把 ADC 值换算成实际母线电压。 */
#define FOC_SHUNT_OHM                (0.02f)   /* 电流采样电阻，单位欧姆。 */
#define FOC_GAIN                     (6.0f)    /* 电流采样放大倍数。 */
#define FOC_CURR_SIGN_U              (1.0f)    /* U 相电流方向符号，用于适配硬件接线方向。 */
#define FOC_CURR_SIGN_V              (1.0f)    /* V 相电流方向符号，用于适配硬件接线方向。 */
#define FOC_CURR_SIGN_W              (1.0f)    /* W 相电流方向符号，用于适配硬件接线方向。 */
#define FOC_PWM_ARR                  (5311u)   /* TIM1 自动重装载值，决定 PWM 分辨率和周期。 */
#define FOC_PWM_HALF_ARR             (2655u)   /* PWM 中点比较值，等效于零矢量输出。 */

/* ADC 与采样参数 */
#define FOC_ADC_FULL_SCALE           (4096.0f)  /* 12 位 ADC 换算分母，按正点原子例程使用 4096。 */
#define FOC_ADC_VREF                 (3.3f)    /* ADC 基准电压，单位伏。 */
#define FOC_CURRENT_OFFSET_CALIB_SAMPLES (4096u) /* 电流偏置校准采样数，样本越多越稳但启动越慢。 */

/* 保护阈值 */
#define FOC_CURRENT_LIMIT_A          (15.0f)   /* 单相电流限值，触发后直接进入故障态。 */
#define FOC_VBUS_UNDERVOLTAGE_V      (10.0f)   /* 母线太低时，SVPWM 已经不再可信。 */
#define FOC_VBUS_OVERVOLTAGE_V       (30.0f)   /* 母线太高时，先保护再说。 */

/* 开环起转参数 */
#define FOC_ALIGN_VDQ_V              (2.0f)    /* 对齐阶段给定的小幅 d 轴电压，主要用于找“初始角”。 */
#define FOC_OPENLOOP_VDQ_V           (4.0f)    /* 开环旋转时的电压幅值，太小带不动，太大会过流。 */
#define FOC_OPENLOOP_START_FREQ_HZ   (2.0f)    /* 开环起步机械频率，低速起转时先保守一些。 */
#define FOC_OPENLOOP_TARGET_FREQ_HZ  (17.0f)   /* 当前调试阶段希望到达的最高机械频率。 */
#define FOC_OPENLOOP_RAMP_TIME_MS    (2000.0f) /* 从起始频率爬到目标频率所用时间。 */

/* PWM 输出限幅 */
#define FOC_SVPWM_MIN_CCR            (20u)     /* 留一点边界余量，避免比较值贴死到 0。 */
#define FOC_SVPWM_MAX_CCR            (FOC_PWM_ARR - 20u) /* 留一点边界余量，避免比较值贴死到 ARR。 */

/* observer 锁定判定阈值。
 * 这部分只用于“调试判定”，不会影响电机能不能转。
 * 它决定什么时候认为 observer 已经稳定跟随开环角。
 */
#define FOC_OBS_ENABLE_FREQ_HZ       (8.0f)    /* 机械频率太低时，不对 observer 做锁定判定。 */
#define FOC_OBS_LOCK_ERR_DEG         (25.0f)   /* 角差在这个范围内，才认为“有机会锁住”。 */
#define FOC_OBS_LOCK_HOLD_MS         (200u)    /* 条件持续这么久，才把状态置为 locked。 */

#ifdef __cplusplus
}
#endif

#endif /* __FOC_CONFIG_H__ */
