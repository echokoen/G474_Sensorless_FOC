#ifndef __FOC_CONFIG_H__
#define __FOC_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 第一阶段固定控制参数。
 * 为什么这样做：把后续所有控制和保护阈值收敛到单一配置层，
 * 便于先快速验证开环转动，再逐步引入 observer / EKF。
 */
#define FOC_POLE_PAIRS               (4.0f)
#define FOC_RS_OHM                   (0.53f)
#define FOC_LS_H                     (0.0006f)
#define FOC_FLUX_WB                  (0.0078f)
#define FOC_TS_SEC                   (6.25e-5f)
#define FOC_VBUS_RATIO               (25.0f)
#define FOC_SHUNT_OHM                (0.02f)
#define FOC_GAIN                     (6.0f)
#define FOC_CURR_SIGN_U              (1.0f)
#define FOC_CURR_SIGN_V              (1.0f)
#define FOC_PWM_ARR                  (5311u)
#define FOC_PWM_HALF_ARR             (2655u)

#define FOC_ADC_FULL_SCALE           (4095.0f)
#define FOC_ADC_VREF                 (3.3f)
#define FOC_CURRENT_OFFSET_CALIB_SAMPLES (4096u)

#define FOC_CURRENT_LIMIT_A          (15.0f)
#define FOC_VBUS_UNDERVOLTAGE_V      (10.0f)
#define FOC_VBUS_OVERVOLTAGE_V       (30.0f)

#define FOC_ALIGN_VDQ_V              (2.0f)
#define FOC_OPENLOOP_VDQ_V           (4.0f)
#define FOC_OPENLOOP_START_FREQ_HZ   (2.0f)
#define FOC_OPENLOOP_TARGET_FREQ_HZ  (18.0f)
#define FOC_OPENLOOP_RAMP_TIME_MS    (2000.0f)

#define FOC_SVPWM_MIN_CCR            (20u)
#define FOC_SVPWM_MAX_CCR            (FOC_PWM_ARR - 20u)

/* 第二阶段 observer 判定阈值 */
#define FOC_OBS_ENABLE_FREQ_HZ       (8.0f)
#define FOC_OBS_LOCK_ERR_DEG         (25.0f)
#define FOC_OBS_LOCK_HOLD_MS         (200u)

#ifdef __cplusplus
}
#endif

#endif /* __FOC_CONFIG_H__ */
