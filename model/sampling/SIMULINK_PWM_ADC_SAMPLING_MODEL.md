# PWM / ADC Dynamic Sampling Timing Model

## Model Goal

This model verifies the PWM and ADC dynamic sampling timing used by the current STM32G474 Sensorless PMSM FOC project.

It is not a current-loop model, not an observer model, and not a motor model. It only checks:

- SVPWM sector;
- duty and CCR generation;
- trusted two-phase selection for three-shunt low-side sampling;
- ADC sample CCR selection;
- safe sampling window width;
- fallback region;
- hold-last request condition.

The implementation is a MATLAB script model because this task is mainly a sweep/report problem over angle and modulation index.

## Files

- `pwm_adc_sampling_params.m`
- `run_pwm_adc_sampling_timing_tests.m`
- `pwm_adc_sampling_test_results.mat`
- `figures/`

## C Code Mapping

The SVPWM calculation follows:

- `Control/Modulation/foc_svpwm.c`

The trusted phase pair follows:

- sector 1 / 6: trusted V/W, reconstruct U;
- sector 2 / 3: trusted U/W, reconstruct V;
- sector 4 / 5: trusted U/V, reconstruct W.

This matches:

- `Control/Modulation/foc_svpwm.c`
- `Control/Sensing/foc_current_sense.c`

The ADC sample timing follows `foc_svpwm_update_adc_trigger_ccr()`:

```text
pair_max = max(ccr_trusted_1, ccr_trusted_2)
safe_start = pair_max + FOC_ADC_TRIG_MARGIN_CCR
safe_end = FOC_ADC_SAMPLE_MAX_CCR
safe_width = safe_end - safe_start
```

If the safe window is wide enough, the algorithm prefers the PWM center point when it is still safely inside the window. Otherwise it moves to the center of the safe window.

```text
if safe_width >= FOC_ADC_TRIG_MIN_WINDOW_CCR:
    if center is safely inside [safe_start, safe_end]:
        adc_sample_ccr = FOC_PWM_HALF_ARR
    else:
        adc_sample_ccr = safe_start + safe_width / 2
    fallback = 0
else:
    adc_sample_ccr = FOC_ADC_TRIG_FALLBACK_CCR
    fallback = 1
```

`hold_last_flag` is asserted when fallback happens and `FOC_ADC_HOLD_LAST_ON_FALLBACK` is enabled.

## Parameters

Parameters are in `pwm_adc_sampling_params.m`:

```matlab
pwm_arr = 5311;
pwm_half_arr = 2655;
adc_margin_ccr = 140;
adc_min_window_ccr = 320;
adc_fallback_ccr = pwm_half_arr;
adc_dynamic_sample_enable = 1;
adc_hold_last_on_fallback = 1;
adc_sample_min_ccr = adc_margin_ccr;
adc_sample_max_ccr = pwm_arr - adc_margin_ccr;
svpwm_min_ccr = 20;
svpwm_max_ccr = pwm_arr - 20;
vbus = 24.0;
modulation_index_list = 0.05:0.05:0.95;
theta_vec = linspace(0, 2*pi, 721);
```

## Run

```matlab
addpath('D:/final_projects/G474_Sensorless_FOC/model/sampling');
results = run_pwm_adc_sampling_timing_tests;
```

## Outputs

The result file is:

```text
model/sampling/pwm_adc_sampling_test_results.mat
```

The script saves figures to:

```text
model/sampling/figures/
```

Generated plots include:

- sector vs angle;
- CCR U/V/W vs angle;
- ADC sample CCR vs angle;
- safe window width vs angle;
- fallback flag vs angle;
- fallback heatmap over modulation index and angle.

## Summary From Current Run

Current full sweep:

```text
total_points = 13699
valid_points = 13675
fallback_points = 24
fallback_ratio = 0.0018
min_safe_width = 191 CCR
worst_angle_deg = 60.00
worst_modulation = 0.95
```

This means the current dynamic sampling algorithm finds a valid safe sampling point for almost all tested angle/modulation combinations. Fallback mainly appears near high modulation and sector boundaries.

## Current Limits

This model does not include:

- real current waveform;
- op-amp settling;
- ADC sample-and-hold capacitor behavior;
- deadtime;
- MOSFET switching edge noise;
- motor plant;
- current loop;
- observer;
- speed loop.

It is only for validating SVPWM geometry, trusted phase selection, ADC trigger point choice, fallback region, and hold-last request behavior.
