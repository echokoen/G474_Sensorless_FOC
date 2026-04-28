% PMSM current loop discrete model parameters.
%
% Values are copied from the current C project:
% Config/foc_config.h
% Control/Loop/foc_current_loop.c
% Control/Model/foc_pi_controller.c
% Control/Modulation/foc_svpwm.c

if exist('pmsm_current_loop_keep_overrides', 'var') && ...
        (pmsm_current_loop_keep_overrides ~= 0)
    return;
end

Ts = 6.25e-5;
Vbus = 24.0;

pole_pairs = 4.0;
Rs = 0.53;
Ld = 0.0006;
Lq = 0.0006;
flux_wb = 0.0078;
flux = flux_wb;

id_ref_a_default = 0.0;
iq_ref_a_default = 1.0;
id_ref_step_time = 0.001;
id_ref_before = 0.0;
id_ref_after = id_ref_a_default;
iq_ref_step_time = 0.001;
iq_ref_before = 0.0;
iq_ref_after = iq_ref_a_default;
sim_mech_speed_rad_s = 50.0;

iq_kp = 3.770;
iq_ki = 3330.0;
id_kp = iq_kp * 0.5;
id_ki = iq_ki * 0.2;

v_limit = 13.5;
dq_v_limit = 13.5;
vd_cmd_sign = -1.0;
vq_cmd_sign = -1.0;

pwm_arr = 5311.0;
pwm_half_arr = 2655.0;
svpwm_min_ccr = 20.0;
svpwm_max_ccr = pwm_arr - 20.0;

% 0: feed InvPark valpha/vbeta directly to the plant.
% 1: feed valpha/vbeta through the C-like SVPWM/inverter equivalent first.
use_svpwm_equivalent = 0;

% This is a plant-side polarity adapter for the current test bench only.
% The C code applies vd_cmd_sign/vq_cmd_sign in foc_current_loop.c because
% the target hardware showed positive Vd/Vq produced negative Id/Iq.
% Keep these signs equal to +1 for textbook dq equations, or -1 to emulate
% the measured hardware voltage/current polarity in this verification model.
plant_vd_effect_sign = -1.0;
plant_vq_effect_sign = -1.0;
