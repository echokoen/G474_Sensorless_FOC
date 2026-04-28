% PWM / ADC dynamic sampling timing model parameters.
%
% Values are copied from:
% Config/foc_config.h
% Control/Modulation/foc_svpwm.c
% Control/Sensing/foc_current_sense.c

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

single_test_modulation = [0.3 0.6 0.9];
