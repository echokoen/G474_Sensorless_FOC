function build_pmsm_observer_pll_block_model()
%BUILD_PMSM_OBSERVER_PLL_BLOCK_MODEL Build a graphical Observer + PLL model.

script_dir = fileparts(mfilename('fullpath'));
model_name = 'pmsm_observer_pll_block_model';
model_path = fullfile(script_dir, [model_name '.slx']);
param_path = fullfile(script_dir, 'pmsm_observer_pll_params.m');

evalin('base', sprintf('run(''%s'');', strrep(param_path, '''', '''''')));

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

if exist(model_path, 'file')
    delete(model_path);
end

new_system(model_name);
open_system(model_name);

init_cmd = [ ...
    'model_file = get_param(bdroot, ''FileName'');' newline ...
    'model_dir = fileparts(model_file);' newline ...
    'run(fullfile(model_dir, ''pmsm_observer_pll_params.m''));'];

set_param(model_name, ...
    'InitFcn', init_cmd, ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'Ts', ...
    'StopTime', '0.15', ...
    'SignalLogging', 'on', ...
    'SaveFormat', 'StructureWithTime');

top_level(model_name);
build_test_stimulus([model_name '/Test_Stimulus']);
build_signal_generator([model_name '/PMSM_SteadyState_Signal_Generator']);
build_flux_observer([model_name '/Flux_Observer']);
build_pll([model_name '/PLL']);
build_error_calc([model_name '/Error_Calculation']);
build_scopes_and_logging([model_name '/Scopes_And_Logging']);
connect_top(model_name);

set_param(model_name, 'SimulationCommand', 'update');
save_system(model_name, model_path);
fprintf('Created %s\n', model_path);
end

function top_level(model_name)
blk(model_name, 'Test_Stimulus', 'simulink/Ports & Subsystems/Subsystem', [40 95 205 210]);
blk(model_name, 'PMSM_SteadyState_Signal_Generator', 'simulink/Ports & Subsystems/Subsystem', [300 70 565 240]);
blk(model_name, 'Flux_Observer', 'simulink/Ports & Subsystems/Subsystem', [670 75 880 230]);
blk(model_name, 'PLL', 'simulink/Ports & Subsystems/Subsystem', [980 70 1190 250]);
blk(model_name, 'Error_Calculation', 'simulink/Ports & Subsystems/Subsystem', [1285 95 1495 235]);
blk(model_name, 'Scopes_And_Logging', 'simulink/Ports & Subsystems/Subsystem', [1590 40 1840 330]);
end

function connect_top(model_name)
ln(model_name, 'Test_Stimulus/1', 'PMSM_SteadyState_Signal_Generator/1');
ln(model_name, 'Test_Stimulus/2', 'PMSM_SteadyState_Signal_Generator/2');
ln(model_name, 'Test_Stimulus/3', 'PMSM_SteadyState_Signal_Generator/3');

ln(model_name, 'PMSM_SteadyState_Signal_Generator/1', 'Flux_Observer/1');
ln(model_name, 'PMSM_SteadyState_Signal_Generator/2', 'Flux_Observer/2');
ln(model_name, 'PMSM_SteadyState_Signal_Generator/3', 'Flux_Observer/3');
ln(model_name, 'PMSM_SteadyState_Signal_Generator/4', 'Flux_Observer/4');

ln(model_name, 'Flux_Observer/3', 'PLL/1');
ln(model_name, 'Flux_Observer/4', 'PLL/2');
ln(model_name, 'Flux_Observer/5', 'PLL/3');

ln(model_name, 'PMSM_SteadyState_Signal_Generator/5', 'Error_Calculation/1');
ln(model_name, 'PLL/5', 'Error_Calculation/2');
ln(model_name, 'PLL/6', 'Error_Calculation/3');
ln(model_name, 'PMSM_SteadyState_Signal_Generator/6', 'Error_Calculation/4');
ln(model_name, 'PLL/4', 'Error_Calculation/5');

% Logging subsystem inputs:
% 1 valpha, 2 vbeta, 3 ialpha, 4 ibeta, 5 theta_real, 6 speed_real,
% 7 flux_alpha, 8 flux_beta, 9 flux_mag, 10 pll_err, 11 pll_integrator,
% 12 speed_raw, 13 speed_obs, 14 theta_raw, 15 theta_comp,
% 16 theta_err_raw_deg, 17 theta_err_comp_deg, 18 speed_err_rad_s,
% 19 flux_limit_hit, 20 speed_limit_hit, 21 integral_limit_hit.
ln(model_name, 'PMSM_SteadyState_Signal_Generator/1', 'Scopes_And_Logging/1');
ln(model_name, 'PMSM_SteadyState_Signal_Generator/2', 'Scopes_And_Logging/2');
ln(model_name, 'PMSM_SteadyState_Signal_Generator/3', 'Scopes_And_Logging/3');
ln(model_name, 'PMSM_SteadyState_Signal_Generator/4', 'Scopes_And_Logging/4');
ln(model_name, 'PMSM_SteadyState_Signal_Generator/5', 'Scopes_And_Logging/5');
ln(model_name, 'PMSM_SteadyState_Signal_Generator/6', 'Scopes_And_Logging/6');
ln(model_name, 'Flux_Observer/3', 'Scopes_And_Logging/7');
ln(model_name, 'Flux_Observer/4', 'Scopes_And_Logging/8');
ln(model_name, 'Flux_Observer/5', 'Scopes_And_Logging/9');
ln(model_name, 'PLL/1', 'Scopes_And_Logging/10');
ln(model_name, 'PLL/2', 'Scopes_And_Logging/11');
ln(model_name, 'PLL/3', 'Scopes_And_Logging/12');
ln(model_name, 'PLL/4', 'Scopes_And_Logging/13');
ln(model_name, 'PLL/5', 'Scopes_And_Logging/14');
ln(model_name, 'PLL/6', 'Scopes_And_Logging/15');
ln(model_name, 'Error_Calculation/1', 'Scopes_And_Logging/16');
ln(model_name, 'Error_Calculation/2', 'Scopes_And_Logging/17');
ln(model_name, 'Error_Calculation/3', 'Scopes_And_Logging/18');
ln(model_name, 'Flux_Observer/6', 'Scopes_And_Logging/19');
ln(model_name, 'PLL/7', 'Scopes_And_Logging/20');
ln(model_name, 'PLL/8', 'Scopes_And_Logging/21');
end

function build_test_stimulus(sys)
clear_subsystem(sys);
outp(sys, 'mech_speed_rad_s', 1, [230 45 260 65]);
outp(sys, 'id_a', 2, [230 100 260 120]);
outp(sys, 'iq_a', 3, [230 155 260 175]);
blk(sys, 'mech_speed_const', 'simulink/Sources/Constant', [45 40 155 70], 'Value', 'sim_mech_speed_rad_s');
blk(sys, 'id_const', 'simulink/Sources/Constant', [45 95 155 125], 'Value', 'sim_id_a');
blk(sys, 'iq_const', 'simulink/Sources/Constant', [45 150 155 180], 'Value', 'sim_iq_a');
ln(sys, 'mech_speed_const/1', 'mech_speed_rad_s/1');
ln(sys, 'id_const/1', 'id_a/1');
ln(sys, 'iq_const/1', 'iq_a/1');
end

function build_signal_generator(sys)
clear_subsystem(sys);
inp(sys, 'mech_speed_rad_s', 1, [25 55 55 75]);
inp(sys, 'id_a', 2, [25 135 55 155]);
inp(sys, 'iq_a', 3, [25 215 55 235]);
outp(sys, 'valpha', 1, [870 45 900 65]);
outp(sys, 'vbeta', 2, [870 95 900 115]);
outp(sys, 'ialpha', 3, [870 180 900 200]);
outp(sys, 'ibeta', 4, [870 230 900 250]);
outp(sys, 'theta_real', 5, [870 300 900 320]);
outp(sys, 'speed_real', 6, [870 350 900 370]);

blk(sys, 'pole_pairs', 'simulink/Math Operations/Gain', [95 50 165 80], 'Gain', 'pole_pairs');
blk(sys, 'theta_integrator', 'simulink/Discrete/Discrete-Time Integrator', [215 50 285 80], ...
    'InitialCondition', '0', 'SampleTime', 'Ts');
wrap2pi(sys, 'wrap_theta_real', [325 45 430 85]);
trig(sys, 'sin_theta', 'sin', [470 25 525 55]);
trig(sys, 'cos_theta', 'cos', [470 75 525 105]);

gain(sys, 'Rs_id', 'Rs', [95 125 155 155]);
gain(sys, 'Rs_iq', 'Rs', [95 205 155 235]);
prod(sys, 'we_iq', [220 145 255 180]);
gain(sys, 'Lq_we_iq', 'Lq', [300 145 360 175]);
sumblk(sys, 'vd_sum', '+-', [405 125 435 175]);
prod(sys, 'we_id', [220 235 255 270]);
gain(sys, 'Ld_we_id', 'Ld', [300 235 360 265]);
gain(sys, 'flux_we', 'flux_wb', [300 290 360 320]);
sumblk(sys, 'vq_sum', '+++', [405 215 435 290]);

prod(sys, 'vd_cos', [565 55 600 90]);
prod(sys, 'vq_sin', [565 105 600 140]);
sumblk(sys, 'valpha_sum', '+-', [650 70 680 125]);
prod(sys, 'vd_sin', [565 165 600 200]);
prod(sys, 'vq_cos', [565 215 600 250]);
sumblk(sys, 'vbeta_sum', '++', [650 185 680 240]);

prod(sys, 'id_cos', [565 300 600 335]);
prod(sys, 'iq_sin', [565 350 600 385]);
sumblk(sys, 'ialpha_sum', '+-', [650 315 680 370]);
prod(sys, 'id_sin', [565 420 600 455]);
prod(sys, 'iq_cos', [565 470 600 505]);
sumblk(sys, 'ibeta_sum', '++', [650 435 680 490]);

ln(sys, 'mech_speed_rad_s/1', 'pole_pairs/1');
ln(sys, 'pole_pairs/1', 'theta_integrator/1');
ln(sys, 'pole_pairs/1', 'speed_real/1');
ln(sys, 'theta_integrator/1', 'wrap_theta_real/1');
ln(sys, 'wrap_theta_real/1', 'theta_real/1');
ln(sys, 'wrap_theta_real/1', 'sin_theta/1');
ln(sys, 'wrap_theta_real/1', 'cos_theta/1');

ln(sys, 'id_a/1', 'Rs_id/1');
ln(sys, 'iq_a/1', 'Rs_iq/1');
ln(sys, 'pole_pairs/1', 'we_iq/1');
ln(sys, 'iq_a/1', 'we_iq/2');
ln(sys, 'we_iq/1', 'Lq_we_iq/1');
ln(sys, 'Rs_id/1', 'vd_sum/1');
ln(sys, 'Lq_we_iq/1', 'vd_sum/2');
ln(sys, 'pole_pairs/1', 'we_id/1');
ln(sys, 'id_a/1', 'we_id/2');
ln(sys, 'we_id/1', 'Ld_we_id/1');
ln(sys, 'pole_pairs/1', 'flux_we/1');
ln(sys, 'Rs_iq/1', 'vq_sum/1');
ln(sys, 'Ld_we_id/1', 'vq_sum/2');
ln(sys, 'flux_we/1', 'vq_sum/3');

ln(sys, 'vd_sum/1', 'vd_cos/1');
ln(sys, 'cos_theta/1', 'vd_cos/2');
ln(sys, 'vq_sum/1', 'vq_sin/1');
ln(sys, 'sin_theta/1', 'vq_sin/2');
ln(sys, 'vd_cos/1', 'valpha_sum/1');
ln(sys, 'vq_sin/1', 'valpha_sum/2');
ln(sys, 'valpha_sum/1', 'valpha/1');
ln(sys, 'vd_sum/1', 'vd_sin/1');
ln(sys, 'sin_theta/1', 'vd_sin/2');
ln(sys, 'vq_sum/1', 'vq_cos/1');
ln(sys, 'cos_theta/1', 'vq_cos/2');
ln(sys, 'vd_sin/1', 'vbeta_sum/1');
ln(sys, 'vq_cos/1', 'vbeta_sum/2');
ln(sys, 'vbeta_sum/1', 'vbeta/1');

ln(sys, 'id_a/1', 'id_cos/1');
ln(sys, 'cos_theta/1', 'id_cos/2');
ln(sys, 'iq_a/1', 'iq_sin/1');
ln(sys, 'sin_theta/1', 'iq_sin/2');
ln(sys, 'id_cos/1', 'ialpha_sum/1');
ln(sys, 'iq_sin/1', 'ialpha_sum/2');
ln(sys, 'ialpha_sum/1', 'ialpha/1');
ln(sys, 'id_a/1', 'id_sin/1');
ln(sys, 'sin_theta/1', 'id_sin/2');
ln(sys, 'iq_a/1', 'iq_cos/1');
ln(sys, 'cos_theta/1', 'iq_cos/2');
ln(sys, 'id_sin/1', 'ibeta_sum/1');
ln(sys, 'iq_cos/1', 'ibeta_sum/2');
ln(sys, 'ibeta_sum/1', 'ibeta/1');
end

function build_flux_observer(sys)
clear_subsystem(sys);
inp(sys, 'valpha', 1, [25 50 55 70]);
inp(sys, 'vbeta', 2, [25 115 55 135]);
inp(sys, 'ialpha', 3, [25 190 55 210]);
inp(sys, 'ibeta', 4, [25 255 55 275]);
outp(sys, 'flux_s_alpha', 1, [890 45 920 65]);
outp(sys, 'flux_s_beta', 2, [890 100 920 120]);
outp(sys, 'flux_alpha', 3, [890 180 920 200]);
outp(sys, 'flux_beta', 4, [890 235 920 255]);
outp(sys, 'flux_mag', 5, [890 305 920 325]);
outp(sys, 'flux_limit_hit', 6, [890 370 920 390]);

gain(sys, 'Rs_ialpha', 'Rs', [115 180 175 210]);
gain(sys, 'Rs_ibeta', 'Rs', [115 245 175 275]);
sumblk(sys, 'e_alpha', '+-', [220 45 250 95]);
sumblk(sys, 'e_beta', '+-', [220 110 250 160]);
blk(sys, 'flux_s_alpha_int', 'simulink/Discrete/Discrete-Time Integrator', [305 45 385 80], ...
    'InitialCondition', 'flux_wb', 'SampleTime', 'Ts');
blk(sys, 'flux_s_beta_int', 'simulink/Discrete/Discrete-Time Integrator', [305 110 385 145], ...
    'InitialCondition', '0', 'SampleTime', 'Ts');
gain(sys, 'Ls_ialpha', 'Ls', [445 180 505 210]);
gain(sys, 'Ls_ibeta', 'Ls', [445 245 505 275]);
sumblk(sys, 'flux_r_alpha_raw', '+-', [545 160 575 210]);
sumblk(sys, 'flux_r_beta_raw', '+-', [545 225 575 275]);
mathblk(sys, 'flux_alpha_sq', 'square', [620 155 670 185]);
mathblk(sys, 'flux_beta_sq', 'square', [620 220 670 250]);
sumblk(sys, 'flux_sq_sum', '++', [700 185 730 235]);
mathblk(sys, 'flux_mag_sqrt', 'sqrt', [760 190 815 230]);
blk(sys, 'flux_gt_limit', 'simulink/Logic and Bit Operations/Relational Operator', [650 330 700 360], 'Operator', '>');
blk(sys, 'flux_limit_hit_double', 'simulink/Signal Attributes/Data Type Conversion', [740 330 800 360], ...
    'OutDataTypeStr', 'double');
blk(sys, 'flux_limit_const', 'simulink/Sources/Constant', [545 320 610 350], 'Value', 'flux_limit');
blk(sys, 'scale_divide', 'simulink/Math Operations/Divide', [650 390 700 420]);
blk(sys, 'one_const', 'simulink/Sources/Constant', [650 455 700 485], 'Value', '1');
blk(sys, 'scale_switch', 'simulink/Signal Routing/Switch', [740 390 795 440], 'Criteria', 'u2 ~= 0');
prod(sys, 'flux_alpha_limited', [820 155 855 190]);
prod(sys, 'flux_beta_limited', [820 220 855 255]);

ln(sys, 'ialpha/1', 'Rs_ialpha/1');
ln(sys, 'ibeta/1', 'Rs_ibeta/1');
ln(sys, 'valpha/1', 'e_alpha/1');
ln(sys, 'Rs_ialpha/1', 'e_alpha/2');
ln(sys, 'vbeta/1', 'e_beta/1');
ln(sys, 'Rs_ibeta/1', 'e_beta/2');
ln(sys, 'e_alpha/1', 'flux_s_alpha_int/1');
ln(sys, 'e_beta/1', 'flux_s_beta_int/1');
ln(sys, 'flux_s_alpha_int/1', 'flux_s_alpha/1');
ln(sys, 'flux_s_beta_int/1', 'flux_s_beta/1');
ln(sys, 'ialpha/1', 'Ls_ialpha/1');
ln(sys, 'ibeta/1', 'Ls_ibeta/1');
ln(sys, 'flux_s_alpha_int/1', 'flux_r_alpha_raw/1');
ln(sys, 'Ls_ialpha/1', 'flux_r_alpha_raw/2');
ln(sys, 'flux_s_beta_int/1', 'flux_r_beta_raw/1');
ln(sys, 'Ls_ibeta/1', 'flux_r_beta_raw/2');
ln(sys, 'flux_r_alpha_raw/1', 'flux_alpha_sq/1');
ln(sys, 'flux_r_beta_raw/1', 'flux_beta_sq/1');
ln(sys, 'flux_alpha_sq/1', 'flux_sq_sum/1');
ln(sys, 'flux_beta_sq/1', 'flux_sq_sum/2');
ln(sys, 'flux_sq_sum/1', 'flux_mag_sqrt/1');
ln(sys, 'flux_mag_sqrt/1', 'flux_mag/1');
ln(sys, 'flux_mag_sqrt/1', 'flux_gt_limit/1');
ln(sys, 'flux_limit_const/1', 'flux_gt_limit/2');
ln(sys, 'flux_gt_limit/1', 'flux_limit_hit_double/1');
ln(sys, 'flux_limit_hit_double/1', 'flux_limit_hit/1');
ln(sys, 'flux_limit_const/1', 'scale_divide/1');
ln(sys, 'flux_mag_sqrt/1', 'scale_divide/2');
ln(sys, 'scale_divide/1', 'scale_switch/1');
ln(sys, 'flux_gt_limit/1', 'scale_switch/2');
ln(sys, 'one_const/1', 'scale_switch/3');
ln(sys, 'flux_r_alpha_raw/1', 'flux_alpha_limited/1');
ln(sys, 'scale_switch/1', 'flux_alpha_limited/2');
ln(sys, 'flux_r_beta_raw/1', 'flux_beta_limited/1');
ln(sys, 'scale_switch/1', 'flux_beta_limited/2');
ln(sys, 'flux_alpha_limited/1', 'flux_alpha/1');
ln(sys, 'flux_beta_limited/1', 'flux_beta/1');
end

function build_pll(sys)
clear_subsystem(sys);
inp(sys, 'flux_alpha', 1, [25 60 55 80]);
inp(sys, 'flux_beta', 2, [25 130 55 150]);
inp(sys, 'flux_mag', 3, [25 205 55 225]);
outp(sys, 'pll_err', 1, [1040 40 1070 60]);
outp(sys, 'pll_integrator', 2, [1040 95 1070 115]);
outp(sys, 'speed_raw', 3, [1040 150 1070 170]);
outp(sys, 'speed_obs', 4, [1040 205 1070 225]);
outp(sys, 'theta_raw', 5, [1040 270 1070 290]);
outp(sys, 'theta_comp', 6, [1040 325 1070 345]);
outp(sys, 'speed_limit_hit', 7, [1040 390 1070 410]);
outp(sys, 'integral_limit_hit', 8, [1040 445 1070 465]);

blk(sys, 'theta_z1', 'simulink/Discrete/Unit Delay', [90 300 155 330], 'InitialCondition', '0', 'SampleTime', 'Ts');
trig(sys, 'sin_theta', 'sin', [205 275 260 305]);
trig(sys, 'cos_theta', 'cos', [205 335 260 365]);
prod(sys, 'flux_beta_cos', [315 115 350 150]);
prod(sys, 'flux_alpha_sin', [315 175 350 210]);
sumblk(sys, 'pll_err_raw', '+-', [405 140 435 190]);
blk(sys, 'eps_const', 'simulink/Sources/Constant', [315 235 365 265], 'Value', 'obs_eps');
blk(sys, 'max_flux_eps', 'simulink/Math Operations/MinMax', [405 220 455 260], 'Function', 'max', 'Inputs', '2');
blk(sys, 'pll_err_divide', 'simulink/Math Operations/Divide', [500 155 550 205]);
gain(sys, 'ki_gain', 'pll_ki', [600 155 660 185]);
gain(sys, 'Ts_gain_integral', 'Ts', [705 155 765 185]);
sumblk(sys, 'pll_integral_sum', '++', [805 150 835 190]);
blk(sys, 'pll_integral_sat', 'simulink/Discontinuities/Saturation', [870 150 930 190], ...
    'UpperLimit', 'pll_integral_limit', 'LowerLimit', '-pll_integral_limit');
blk(sys, 'pll_integral_z1', 'simulink/Discrete/Unit Delay', [805 95 865 125], 'InitialCondition', '0', 'SampleTime', 'Ts');
gain(sys, 'kp_gain', 'pll_kp', [600 235 660 265]);
sumblk(sys, 'speed_sum', '++', [705 220 735 270]);
blk(sys, 'speed_saturation', 'simulink/Discontinuities/Saturation', [780 220 850 270], ...
    'UpperLimit', 'obs_speed_limit_rad_s', 'LowerLimit', '-obs_speed_limit_rad_s');

blk(sys, 'speed_obs_z1', 'simulink/Discrete/Unit Delay', [600 335 660 365], 'InitialCondition', '0', 'SampleTime', 'Ts');
sumblk(sys, 'speed_filter_err', '+-', [705 330 735 380]);
gain(sys, 'speed_filter_alpha', 'obs_speed_filter_alpha', [780 335 845 365]);
sumblk(sys, 'speed_filter_sum', '++', [890 330 920 380]);

gain(sys, 'Ts_gain_theta', 'Ts', [600 435 660 465]);
sumblk(sys, 'theta_sum', '++', [705 425 735 475]);
wrap2pi(sys, 'wrap_theta_raw', [780 425 875 475]);
blk(sys, 'theta_comp_const', 'simulink/Sources/Constant', [780 515 855 545], 'Value', 'obs_theta_comp_rad');
sumblk(sys, 'theta_comp_sum', '++', [890 485 920 535]);
wrap2pi(sys, 'wrap_theta_comp', [950 485 1030 535]);

absblk(sys, 'abs_speed_raw', [870 570 920 600]);
blk(sys, 'speed_limit_threshold', 'simulink/Sources/Constant', [870 625 950 655], 'Value', 'obs_speed_limit_rad_s * 0.98');
blk(sys, 'speed_limit_compare', 'simulink/Logic and Bit Operations/Relational Operator', [980 585 1030 625], 'Operator', '>=');
blk(sys, 'speed_limit_hit_double', 'simulink/Signal Attributes/Data Type Conversion', [1045 590 1105 620], ...
    'OutDataTypeStr', 'double');
absblk(sys, 'abs_integral', [870 680 920 710]);
blk(sys, 'integral_limit_threshold', 'simulink/Sources/Constant', [870 735 950 765], 'Value', 'pll_integral_limit * 0.98');
blk(sys, 'integral_limit_compare', 'simulink/Logic and Bit Operations/Relational Operator', [980 695 1030 735], 'Operator', '>=');
blk(sys, 'integral_limit_hit_double', 'simulink/Signal Attributes/Data Type Conversion', [1045 700 1105 730], ...
    'OutDataTypeStr', 'double');

ln(sys, 'theta_z1/1', 'sin_theta/1');
ln(sys, 'theta_z1/1', 'cos_theta/1');
ln(sys, 'flux_beta/1', 'flux_beta_cos/1');
ln(sys, 'cos_theta/1', 'flux_beta_cos/2');
ln(sys, 'flux_alpha/1', 'flux_alpha_sin/1');
ln(sys, 'sin_theta/1', 'flux_alpha_sin/2');
ln(sys, 'flux_beta_cos/1', 'pll_err_raw/1');
ln(sys, 'flux_alpha_sin/1', 'pll_err_raw/2');
ln(sys, 'flux_mag/1', 'max_flux_eps/1');
ln(sys, 'eps_const/1', 'max_flux_eps/2');
ln(sys, 'pll_err_raw/1', 'pll_err_divide/1');
ln(sys, 'max_flux_eps/1', 'pll_err_divide/2');
ln(sys, 'pll_err_divide/1', 'pll_err/1');
ln(sys, 'pll_err_divide/1', 'ki_gain/1');
ln(sys, 'ki_gain/1', 'Ts_gain_integral/1');
ln(sys, 'Ts_gain_integral/1', 'pll_integral_sum/1');
ln(sys, 'pll_integral_z1/1', 'pll_integral_sum/2');
ln(sys, 'pll_integral_sum/1', 'pll_integral_sat/1');
ln(sys, 'pll_integral_sat/1', 'pll_integral_z1/1');
ln(sys, 'pll_integral_sat/1', 'pll_integrator/1');
ln(sys, 'pll_err_divide/1', 'kp_gain/1');
ln(sys, 'kp_gain/1', 'speed_sum/1');
ln(sys, 'pll_integral_sat/1', 'speed_sum/2');
ln(sys, 'speed_sum/1', 'speed_saturation/1');
ln(sys, 'speed_saturation/1', 'speed_raw/1');

ln(sys, 'speed_saturation/1', 'speed_filter_err/1');
ln(sys, 'speed_obs_z1/1', 'speed_filter_err/2');
ln(sys, 'speed_filter_err/1', 'speed_filter_alpha/1');
ln(sys, 'speed_filter_alpha/1', 'speed_filter_sum/1');
ln(sys, 'speed_obs_z1/1', 'speed_filter_sum/2');
ln(sys, 'speed_filter_sum/1', 'speed_obs_z1/1');
ln(sys, 'speed_filter_sum/1', 'speed_obs/1');

ln(sys, 'speed_saturation/1', 'Ts_gain_theta/1');
ln(sys, 'Ts_gain_theta/1', 'theta_sum/1');
ln(sys, 'theta_z1/1', 'theta_sum/2');
ln(sys, 'theta_sum/1', 'wrap_theta_raw/1');
ln(sys, 'wrap_theta_raw/1', 'theta_z1/1');
ln(sys, 'wrap_theta_raw/1', 'theta_raw/1');
ln(sys, 'wrap_theta_raw/1', 'theta_comp_sum/1');
ln(sys, 'theta_comp_const/1', 'theta_comp_sum/2');
ln(sys, 'theta_comp_sum/1', 'wrap_theta_comp/1');
ln(sys, 'wrap_theta_comp/1', 'theta_comp/1');

ln(sys, 'speed_saturation/1', 'abs_speed_raw/1');
ln(sys, 'abs_speed_raw/1', 'speed_limit_compare/1');
ln(sys, 'speed_limit_threshold/1', 'speed_limit_compare/2');
ln(sys, 'speed_limit_compare/1', 'speed_limit_hit_double/1');
ln(sys, 'speed_limit_hit_double/1', 'speed_limit_hit/1');
ln(sys, 'pll_integral_sat/1', 'abs_integral/1');
ln(sys, 'abs_integral/1', 'integral_limit_compare/1');
ln(sys, 'integral_limit_threshold/1', 'integral_limit_compare/2');
ln(sys, 'integral_limit_compare/1', 'integral_limit_hit_double/1');
ln(sys, 'integral_limit_hit_double/1', 'integral_limit_hit/1');
end

function build_error_calc(sys)
clear_subsystem(sys);
inp(sys, 'theta_real', 1, [25 50 55 70]);
inp(sys, 'theta_raw', 2, [25 115 55 135]);
inp(sys, 'theta_comp', 3, [25 180 55 200]);
inp(sys, 'speed_real', 4, [25 250 55 270]);
inp(sys, 'speed_obs', 5, [25 315 55 335]);
outp(sys, 'theta_err_raw_deg', 1, [570 95 600 115]);
outp(sys, 'theta_err_comp_deg', 2, [570 175 600 195]);
outp(sys, 'speed_err_rad_s', 3, [570 290 600 310]);

sumblk(sys, 'theta_raw_minus_real', '+-', [120 95 150 145]);
wrap_pi(sys, 'wrap_raw_err', [195 95 285 145]);
gain(sys, 'raw_rad_to_deg', '180/pi', [335 100 415 130]);
sumblk(sys, 'theta_comp_minus_real', '+-', [120 175 150 225]);
wrap_pi(sys, 'wrap_comp_err', [195 175 285 225]);
gain(sys, 'comp_rad_to_deg', '180/pi', [335 180 415 210]);
sumblk(sys, 'speed_obs_minus_real', '+-', [230 285 260 335]);

ln(sys, 'theta_raw/1', 'theta_raw_minus_real/1');
ln(sys, 'theta_real/1', 'theta_raw_minus_real/2');
ln(sys, 'theta_raw_minus_real/1', 'wrap_raw_err/1');
ln(sys, 'wrap_raw_err/1', 'raw_rad_to_deg/1');
ln(sys, 'raw_rad_to_deg/1', 'theta_err_raw_deg/1');
ln(sys, 'theta_comp/1', 'theta_comp_minus_real/1');
ln(sys, 'theta_real/1', 'theta_comp_minus_real/2');
ln(sys, 'theta_comp_minus_real/1', 'wrap_comp_err/1');
ln(sys, 'wrap_comp_err/1', 'comp_rad_to_deg/1');
ln(sys, 'comp_rad_to_deg/1', 'theta_err_comp_deg/1');
ln(sys, 'speed_obs/1', 'speed_obs_minus_real/1');
ln(sys, 'speed_real/1', 'speed_obs_minus_real/2');
ln(sys, 'speed_obs_minus_real/1', 'speed_err_rad_s/1');
end

function build_scopes_and_logging(sys)
clear_subsystem(sys);
names = {'valpha','vbeta','ialpha','ibeta','theta_real','speed_real','flux_alpha','flux_beta','flux_mag', ...
         'pll_err','pll_integrator','speed_raw','speed_obs','theta_raw','theta_comp', ...
         'theta_err_raw_deg','theta_err_comp_deg','speed_err_rad_s','flux_limit_hit','speed_limit_hit','integral_limit_hit'};
for k = 1:numel(names)
    inp(sys, names{k}, k, [25 20 + 30*k 55 40 + 30*k]);
end

blk(sys, 'observer_pll_block_mux', 'simulink/Signal Routing/Mux', [185 45 225 470], 'Inputs', '21');
blk(sys, 'observer_pll_block_log', 'simulink/Sinks/To Workspace', [285 245 430 280], ...
    'VariableName', 'observer_pll_block_log', 'SaveFormat', 'Array');

blk(sys, 'angle_mux', 'simulink/Signal Routing/Mux', [500 45 535 115], 'Inputs', '3');
blk(sys, 'Scope_Angle', 'simulink/Sinks/Scope', [575 60 675 100]);
blk(sys, 'speed_mux', 'simulink/Signal Routing/Mux', [500 145 535 215], 'Inputs', '3');
blk(sys, 'Scope_Speed', 'simulink/Sinks/Scope', [575 160 675 200]);
blk(sys, 'error_mux', 'simulink/Signal Routing/Mux', [500 245 535 315], 'Inputs', '3');
blk(sys, 'Scope_Error', 'simulink/Sinks/Scope', [575 260 675 300]);
blk(sys, 'flux_mux', 'simulink/Signal Routing/Mux', [500 345 535 415], 'Inputs', '3');
blk(sys, 'Scope_Flux', 'simulink/Sinks/Scope', [575 360 675 400]);
blk(sys, 'pll_mux', 'simulink/Signal Routing/Mux', [500 445 535 500], 'Inputs', '2');
blk(sys, 'Scope_PLL', 'simulink/Sinks/Scope', [575 455 675 495]);

for k = 1:numel(names)
    ln(sys, sprintf('%s/1', names{k}), sprintf('observer_pll_block_mux/%d', k));
end
ln(sys, 'observer_pll_block_mux/1', 'observer_pll_block_log/1');

ln(sys, 'theta_real/1', 'angle_mux/1');
ln(sys, 'theta_raw/1', 'angle_mux/2');
ln(sys, 'theta_comp/1', 'angle_mux/3');
ln(sys, 'angle_mux/1', 'Scope_Angle/1');
ln(sys, 'speed_real/1', 'speed_mux/1');
ln(sys, 'speed_raw/1', 'speed_mux/2');
ln(sys, 'speed_obs/1', 'speed_mux/3');
ln(sys, 'speed_mux/1', 'Scope_Speed/1');
ln(sys, 'theta_err_raw_deg/1', 'error_mux/1');
ln(sys, 'theta_err_comp_deg/1', 'error_mux/2');
ln(sys, 'speed_err_rad_s/1', 'error_mux/3');
ln(sys, 'error_mux/1', 'Scope_Error/1');
ln(sys, 'flux_alpha/1', 'flux_mux/1');
ln(sys, 'flux_beta/1', 'flux_mux/2');
ln(sys, 'flux_mag/1', 'flux_mux/3');
ln(sys, 'flux_mux/1', 'Scope_Flux/1');
ln(sys, 'pll_err/1', 'pll_mux/1');
ln(sys, 'pll_integrator/1', 'pll_mux/2');
ln(sys, 'pll_mux/1', 'Scope_PLL/1');
end

function clear_subsystem(sys)
Simulink.SubSystem.deleteContents(sys);
end

function inp(sys, name, port, pos)
blk(sys, name, 'simulink/Sources/In1', pos, 'Port', num2str(port));
end

function outp(sys, name, port, pos)
blk(sys, name, 'simulink/Sinks/Out1', pos, 'Port', num2str(port));
end

function gain(sys, name, g, pos)
blk(sys, name, 'simulink/Math Operations/Gain', pos, 'Gain', g);
end

function prod(sys, name, pos)
blk(sys, name, 'simulink/Math Operations/Product', pos);
end

function sumblk(sys, name, inputs, pos)
blk(sys, name, 'simulink/Math Operations/Sum', pos, 'Inputs', inputs);
end

function trig(sys, name, op, pos)
blk(sys, name, 'simulink/Math Operations/Trigonometric Function', pos, 'Operator', op);
end

function mathblk(sys, name, op, pos)
blk(sys, name, 'simulink/Math Operations/Math Function', pos, 'Operator', op);
end

function absblk(sys, name, pos)
blk(sys, name, 'simulink/Math Operations/Abs', pos);
end

function wrap2pi(sys, name, pos)
blk(sys, name, 'simulink/User-Defined Functions/MATLAB Function', pos);
set_mfunc([sys '/' name], ['function y = fcn(u)' newline '%#codegen' newline ...
    'y = u - floor(u / 6.283185307179586) * 6.283185307179586;' newline 'end']);
end

function wrap_pi(sys, name, pos)
blk(sys, name, 'simulink/User-Defined Functions/MATLAB Function', pos);
set_mfunc([sys '/' name], ['function y = fcn(u)' newline '%#codegen' newline ...
    'y = u;' newline ...
    'while y > 3.141592653589793' newline ...
    '    y = y - 6.283185307179586;' newline ...
    'end' newline ...
    'while y < -3.141592653589793' newline ...
    '    y = y + 6.283185307179586;' newline ...
    'end' newline 'end']);
end

function set_mfunc(block_path, script_text)
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
chart.Script = script_text;
end

function blk(sys, name, lib, pos, varargin)
add_block(lib, [sys '/' name], 'Position', pos, varargin{:});
end

function ln(sys, src, dst)
add_line(sys, src, dst, 'autorouting', 'on');
end
