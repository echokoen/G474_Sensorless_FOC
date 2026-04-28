function build_pwm_adc_sampling_timing_model()
%BUILD_PWM_ADC_SAMPLING_TIMING_MODEL Build block-based PWM/ADC timing model.

script_dir = fileparts(mfilename('fullpath'));
model_name = 'pwm_adc_sampling_timing';
model_path = fullfile(script_dir, [model_name '.slx']);
param_path = fullfile(script_dir, 'pwm_adc_sampling_params.m');

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
    'run(fullfile(model_dir, ''pwm_adc_sampling_params.m''));'];

set_param(model_name, 'InitFcn', init_cmd, ...
    'Solver', 'FixedStepDiscrete', 'FixedStep', '1', 'StopTime', '0');

build_top(model_name);
build_voltage_vector([model_name '/Voltage_Vector_Generator']);
build_svpwm([model_name '/SVPWM_CodeLike']);
build_trusted_pair([model_name '/TrustedPair_Selector']);
build_sampling_window([model_name '/SamplingWindow_Calc']);
build_diagnostics([model_name '/Sampling_Diagnostics']);
build_logging([model_name '/Scopes_And_Logging']);
connect_top(model_name);

set_param(model_name, 'SimulationCommand', 'update');
save_system(model_name, model_path);
fprintf('Created %s\n', model_path);
end

function build_top(m)
blk(m, 'modulation_index', 'simulink/Sources/Constant', [40 55 145 85], 'Value', '0.6');
blk(m, 'theta_vec_rad', 'simulink/Sources/Constant', [40 130 145 160], 'Value', 'pi/6');
blk(m, 'vbus', 'simulink/Sources/Constant', [40 205 145 235], 'Value', 'vbus');
blk(m, 'Voltage_Vector_Generator', 'simulink/Ports & Subsystems/Subsystem', [235 70 445 205]);
blk(m, 'SVPWM_CodeLike', 'simulink/Ports & Subsystems/Subsystem', [535 55 760 250]);
blk(m, 'TrustedPair_Selector', 'simulink/Ports & Subsystems/Subsystem', [850 75 1085 240]);
blk(m, 'SamplingWindow_Calc', 'simulink/Ports & Subsystems/Subsystem', [1180 80 1435 265]);
blk(m, 'Sampling_Diagnostics', 'simulink/Ports & Subsystems/Subsystem', [1525 115 1740 240]);
blk(m, 'Scopes_And_Logging', 'simulink/Ports & Subsystems/Subsystem', [1840 55 2080 340]);
end

function connect_top(m)
ln(m, 'modulation_index/1', 'Voltage_Vector_Generator/1');
ln(m, 'theta_vec_rad/1', 'Voltage_Vector_Generator/2');
ln(m, 'vbus/1', 'Voltage_Vector_Generator/3');
ln(m, 'Voltage_Vector_Generator/1', 'SVPWM_CodeLike/1');
ln(m, 'Voltage_Vector_Generator/2', 'SVPWM_CodeLike/2');
ln(m, 'vbus/1', 'SVPWM_CodeLike/3');
ln(m, 'theta_vec_rad/1', 'SVPWM_CodeLike/4');
ln(m, 'SVPWM_CodeLike/1', 'TrustedPair_Selector/1');
ln(m, 'SVPWM_CodeLike/5', 'TrustedPair_Selector/2');
ln(m, 'SVPWM_CodeLike/6', 'TrustedPair_Selector/3');
ln(m, 'SVPWM_CodeLike/7', 'TrustedPair_Selector/4');
ln(m, 'TrustedPair_Selector/5', 'SamplingWindow_Calc/1');
ln(m, 'TrustedPair_Selector/6', 'SamplingWindow_Calc/2');
ln(m, 'SamplingWindow_Calc/5', 'Sampling_Diagnostics/1');
ln(m, 'SamplingWindow_Calc/3', 'Sampling_Diagnostics/2');

signals = {'Voltage_Vector_Generator/1','Voltage_Vector_Generator/2', ...
    'SVPWM_CodeLike/1','SVPWM_CodeLike/2','SVPWM_CodeLike/3','SVPWM_CodeLike/4','SVPWM_CodeLike/5','SVPWM_CodeLike/6','SVPWM_CodeLike/7', ...
    'TrustedPair_Selector/1','TrustedPair_Selector/2','TrustedPair_Selector/3','TrustedPair_Selector/4', ...
    'SamplingWindow_Calc/1','SamplingWindow_Calc/2','SamplingWindow_Calc/3','SamplingWindow_Calc/4','SamplingWindow_Calc/5', ...
    'Sampling_Diagnostics/1','Sampling_Diagnostics/2'};
for k = 1:numel(signals)
    ln(m, signals{k}, sprintf('Scopes_And_Logging/%d', k));
end
end

function build_voltage_vector(s)
clear_subsystem(s);
inp(s, 'modulation_index', 1, [25 45 55 65]);
inp(s, 'theta_vec_rad', 2, [25 110 55 130]);
inp(s, 'vbus', 3, [25 175 55 195]);
outp(s, 'valpha', 1, [575 75 605 95]);
outp(s, 'vbeta', 2, [575 160 605 180]);
prod(s, 'm_times_vbus', [95 75 130 110]);
gain(s, 'one_over_sqrt3', '1/sqrt(3)', [165 80 245 105]);
trig(s, 'cos_theta', 'cos', [165 130 220 160]);
trig(s, 'sin_theta', 'sin', [165 200 220 230]);
prod(s, 'valpha_product', [320 80 355 115]);
prod(s, 'vbeta_product', [320 165 355 200]);
ln(s, 'modulation_index/1', 'm_times_vbus/1');
ln(s, 'vbus/1', 'm_times_vbus/2');
ln(s, 'm_times_vbus/1', 'one_over_sqrt3/1');
ln(s, 'theta_vec_rad/1', 'cos_theta/1');
ln(s, 'theta_vec_rad/1', 'sin_theta/1');
ln(s, 'one_over_sqrt3/1', 'valpha_product/1');
ln(s, 'cos_theta/1', 'valpha_product/2');
ln(s, 'one_over_sqrt3/1', 'vbeta_product/1');
ln(s, 'sin_theta/1', 'vbeta_product/2');
ln(s, 'valpha_product/1', 'valpha/1');
ln(s, 'vbeta_product/1', 'vbeta/1');
end

function build_svpwm(s)
clear_subsystem(s);
inp(s, 'valpha', 1, [25 45 55 65]);
inp(s, 'vbeta', 2, [25 100 55 120]);
inp(s, 'vbus', 3, [25 155 55 175]);
inp(s, 'theta_vec_rad', 4, [25 215 55 235]);
outp(s, 'sector', 1, [1160 35 1190 55]);

gain(s, 'theta_to_sector_width', '3/pi', [95 210 165 240]);
mathblk(s, 'sector_floor', 'floor', [200 210 260 240]);
blk(s, 'sector_plus_one', 'simulink/Math Operations/Bias', [300 210 360 240], 'Bias', '1');
blk(s, 'sector_sat', 'simulink/Discontinuities/Saturation', [395 210 455 240], 'LowerLimit', '1', 'UpperLimit', '6');
ln(s, 'theta_vec_rad/1', 'theta_to_sector_width/1');
ln(s, 'theta_to_sector_width/1', 'sector_floor/1');
ln(s, 'sector_floor/1', 'sector_plus_one/1');
ln(s, 'sector_plus_one/1', 'sector_sat/1');
ln(s, 'sector_sat/1', 'sector/1');

sumblk(s, 'sector_minus_one', '+-', [500 210 530 250]);
blk(s, 'one_const', 'simulink/Sources/Constant', [455 260 500 290], 'Value', '1');
gain(s, 'sector_angle_gain', 'pi/3', [565 215 635 245]);
sumblk(s, 'theta_sector_raw', '+-', [675 205 705 255]);
blk(s, 'theta_sector_sat', 'simulink/Discontinuities/Saturation', [740 210 810 245], 'LowerLimit', '0', 'UpperLimit', 'pi/3');
ln(s, 'sector_sat/1', 'sector_minus_one/1');
ln(s, 'one_const/1', 'sector_minus_one/2');
ln(s, 'sector_minus_one/1', 'sector_angle_gain/1');
ln(s, 'theta_vec_rad/1', 'theta_sector_raw/1');
ln(s, 'sector_angle_gain/1', 'theta_sector_raw/2');
ln(s, 'theta_sector_raw/1', 'theta_sector_sat/1');

mathblk(s, 'valpha_sq', 'square', [95 25 145 55]);
mathblk(s, 'vbeta_sq', 'square', [95 85 145 115]);
sumblk(s, 'vref_sq_sum', '++', [180 50 210 100]);
mathblk(s, 'vref_sqrt', 'sqrt', [245 55 300 95]);
blk(s, 'vref_div_vbus', 'simulink/Math Operations/Divide', [340 60 390 110]);
gain(s, 'common_gain', 'pwm_arr*sqrt(3)', [430 65 540 100]);
blk(s, 'pi_over_3_const', 'simulink/Sources/Constant', [835 135 900 165], 'Value', 'pi/3');
sumblk(s, 'pi3_minus_theta_sector', '+-', [930 125 960 175]);
trig(s, 'sin_pi3_minus', 'sin', [995 130 1050 160]);
trig(s, 'sin_theta_sector', 'sin', [995 210 1050 240]);
prod(s, 't1_raw', [1080 125 1115 160]);
prod(s, 't2_raw', [1080 210 1115 245]);
sumblk(s, 't_sum', '++', [1115 470 1145 520]);
blk(s, 'pwm_arr_const_scale', 'simulink/Sources/Constant', [1115 545 1180 575], 'Value', 'pwm_arr');
blk(s, 'scale_divide', 'simulink/Math Operations/Divide', [1210 510 1260 560]);
blk(s, 'overmod_compare', 'simulink/Logic and Bit Operations/Relational Operator', [1210 610 1260 650], 'Operator', '>');
blk(s, 'scale_one', 'simulink/Sources/Constant', [1210 680 1260 710], 'Value', '1');
blk(s, 'scale_switch', 'simulink/Signal Routing/Switch', [1300 550 1360 610], 'Criteria', 'u2 ~= 0');
prod(s, 't1', [1400 125 1435 160]);
prod(s, 't2', [1400 210 1435 245]);
ln(s, 'valpha/1', 'valpha_sq/1');
ln(s, 'vbeta/1', 'vbeta_sq/1');
ln(s, 'valpha_sq/1', 'vref_sq_sum/1');
ln(s, 'vbeta_sq/1', 'vref_sq_sum/2');
ln(s, 'vref_sq_sum/1', 'vref_sqrt/1');
ln(s, 'vref_sqrt/1', 'vref_div_vbus/1');
ln(s, 'vbus/1', 'vref_div_vbus/2');
ln(s, 'vref_div_vbus/1', 'common_gain/1');
ln(s, 'pi_over_3_const/1', 'pi3_minus_theta_sector/1');
ln(s, 'theta_sector_sat/1', 'pi3_minus_theta_sector/2');
ln(s, 'pi3_minus_theta_sector/1', 'sin_pi3_minus/1');
ln(s, 'theta_sector_sat/1', 'sin_theta_sector/1');
ln(s, 'common_gain/1', 't1_raw/1');
ln(s, 'sin_pi3_minus/1', 't1_raw/2');
ln(s, 'common_gain/1', 't2_raw/1');
ln(s, 'sin_theta_sector/1', 't2_raw/2');
ln(s, 't1_raw/1', 't_sum/1');
ln(s, 't2_raw/1', 't_sum/2');
ln(s, 'pwm_arr_const_scale/1', 'scale_divide/1');
ln(s, 't_sum/1', 'scale_divide/2');
ln(s, 't_sum/1', 'overmod_compare/1');
ln(s, 'pwm_arr_const_scale/1', 'overmod_compare/2');
ln(s, 'scale_divide/1', 'scale_switch/1');
ln(s, 'overmod_compare/1', 'scale_switch/2');
ln(s, 'scale_one/1', 'scale_switch/3');
ln(s, 't1_raw/1', 't1/1');
ln(s, 'scale_switch/1', 't1/2');
ln(s, 't2_raw/1', 't2/1');
ln(s, 'scale_switch/1', 't2/2');

% Duty/CCR mapping uses visible Sum/Product plus native selector helpers.
build_svpwm_phase_time_select(s, 'ta', 't1', 't2', [80 760 1120 850]);
build_svpwm_phase_time_select(s, 'tb', 't1', 't2', [80 900 1120 990]);
build_svpwm_phase_time_select(s, 'tc', 't1', 't2', [80 1040 1120 1130]);
connect_phase_time_select(s, 'ta', 'sector_sat/1', 't1/1', 't2/1');
connect_phase_time_select(s, 'tb', 'sector_sat/1', 't1/1', 't2/1');
connect_phase_time_select(s, 'tc', 'sector_sat/1', 't1/1', 't2/1');

% half_t0 = 0.5 * max(pwm_arr - t1 - t2, 0)
sumblk(s, 't0_raw', '+--', [1480 170 1510 230]);
blk(s, 'pwm_arr_const_t0', 'simulink/Sources/Constant', [1400 330 1470 360], 'Value', 'pwm_arr');
blk(s, 't0_sat', 'simulink/Discontinuities/Saturation', [1545 180 1605 215], 'LowerLimit', '0', 'UpperLimit', 'pwm_arr');
gain(s, 'half_t0', '0.5', [1640 185 1700 215]);
ln(s, 'pwm_arr_const_t0/1', 't0_raw/1');
ln(s, 't1/1', 't0_raw/2');
ln(s, 't2/1', 't0_raw/3');
ln(s, 't0_raw/1', 't0_sat/1');
ln(s, 't0_sat/1', 'half_t0/1');
ln(s, 'half_t0/1', 'ta_half_t0/1');
ln(s, 'half_t0/1', 'tb_half_t0/1');
ln(s, 'half_t0/1', 'tc_half_t0/1');

finish_duty_ccr(s, 'ta_select/1', 'duty_u', 'ccr_u', [900 760], 2, 5);
finish_duty_ccr(s, 'tb_select/1', 'duty_v', 'ccr_v', [900 900], 3, 6);
finish_duty_ccr(s, 'tc_select/1', 'duty_w', 'ccr_w', [900 1040], 4, 7);
end

function build_svpwm_phase_time_select(s, phase, ~, ~, base)
x = base(1); y = base(2);
% candidate names:
% ta: [t1+t2+h, t1+h, h, h, t2+h, t1+t2+h]
% tb: [t2+h, t1+t2+h, t1+t2+h, t1+h, h, h]
% tc: [h, h, t2+h, t1+t2+h, t1+t2+h, t1+h]
sumblk(s, [phase '_t1_t2_h'], '+++', [x y x+35 y+45]);
sumblk(s, [phase '_t1_h'], '++', [x y+50 x+35 y+85]);
sumblk(s, [phase '_t2_h'], '++', [x y+90 x+35 y+125]);
gain(s, [phase '_half_t0'], '1', [x y+135 x+45 y+165]);
select6(s, [phase '_select'], [x+130 y x+360 y+170]);
end

function connect_phase_time_select(s, phase, sector, t1, t2)
ln(s, t1, [phase '_t1_t2_h/1']); ln(s, t2, [phase '_t1_t2_h/2']); ln(s, [phase '_half_t0/1'], [phase '_t1_t2_h/3']);
ln(s, t1, [phase '_t1_h/1']); ln(s, [phase '_half_t0/1'], [phase '_t1_h/2']);
ln(s, t2, [phase '_t2_h/1']); ln(s, [phase '_half_t0/1'], [phase '_t2_h/2']);
ln(s, sector, [phase '_select/1']);
switch phase
    case 'ta'
        src = {[phase '_t1_t2_h/1'], [phase '_t1_h/1'], [phase '_half_t0/1'], [phase '_half_t0/1'], [phase '_t2_h/1'], [phase '_t1_t2_h/1']};
    case 'tb'
        src = {[phase '_t2_h/1'], [phase '_t1_t2_h/1'], [phase '_t1_t2_h/1'], [phase '_t1_h/1'], [phase '_half_t0/1'], [phase '_half_t0/1']};
    otherwise
        src = {[phase '_half_t0/1'], [phase '_half_t0/1'], [phase '_t2_h/1'], [phase '_t1_t2_h/1'], [phase '_t1_t2_h/1'], [phase '_t1_h/1']};
end
for k = 1:6
    ln(s, src{k}, sprintf('%s_select/%d', phase, k+1));
end
end

function select6(s, name, pos)
blk(s, name, 'simulink/Signal Routing/Multiport Switch', pos, 'Inputs', '6', 'DataPortOrder', 'One-based contiguous');
end

function finish_duty_ccr(s, src, duty_name, ccr_name, base, duty_port, ccr_port)
x = base(1); y = base(2);
gain(s, [duty_name '_gain'], '1/pwm_arr', [x y x+70 y+30]);
blk(s, [duty_name '_sat'], 'simulink/Discontinuities/Saturation', [x+105 y x+165 y+30], 'LowerLimit', '0', 'UpperLimit', '1');
gain(s, [ccr_name '_scale'], 'pwm_arr', [x+210 y x+280 y+30]);
mathblk(s, [ccr_name '_floor'], 'floor', [x+315 y x+375 y+30]);
blk(s, [ccr_name '_sat'], 'simulink/Discontinuities/Saturation', [x+410 y x+470 y+30], 'LowerLimit', 'svpwm_min_ccr', 'UpperLimit', 'svpwm_max_ccr');
outp(s, duty_name, duty_port, [x+520 y x+550 y+20]);
outp(s, ccr_name, ccr_port, [x+520 y+40 x+550 y+60]);
ln(s, src, [duty_name '_gain/1']);
ln(s, [duty_name '_gain/1'], [duty_name '_sat/1']);
ln(s, [duty_name '_sat/1'], [duty_name '/1']);
ln(s, [duty_name '_sat/1'], [ccr_name '_scale/1']);
ln(s, [ccr_name '_scale/1'], [ccr_name '_floor/1']);
ln(s, [ccr_name '_floor/1'], [ccr_name '_sat/1']);
ln(s, [ccr_name '_sat/1'], [ccr_name '/1']);
end

function build_trusted_pair(s)
clear_subsystem(s);
inp(s, 'sector', 1, [25 40 55 60]);
inp(s, 'ccr_u', 2, [25 105 55 125]);
inp(s, 'ccr_v', 3, [25 170 55 190]);
inp(s, 'ccr_w', 4, [25 235 55 255]);
outp(s, 'trusted_pair', 1, [710 35 740 55]);
outp(s, 'trusted_phase_a', 2, [710 90 740 110]);
outp(s, 'trusted_phase_b', 3, [710 145 740 165]);
outp(s, 'reconstruct_phase', 4, [710 200 740 220]);
outp(s, 'ccr_x', 5, [710 255 740 275]);
outp(s, 'ccr_y', 6, [710 310 740 330]);
const_select6(s, 'trusted_pair_sel', {'1','2','2','3','3','1'}, [140 30 360 80]);
const_select6(s, 'phase_a_sel', {'2','1','1','1','1','2'}, [140 85 360 135]);
const_select6(s, 'phase_b_sel', {'3','3','3','2','2','3'}, [140 140 360 190]);
const_select6(s, 'reconstruct_sel', {'1','2','2','3','3','1'}, [140 195 360 245]);
signal_select6(s, 'ccr_x_sel', {'ccr_v/1','ccr_u/1','ccr_u/1','ccr_u/1','ccr_u/1','ccr_v/1'}, [140 250 420 320]);
signal_select6(s, 'ccr_y_sel', {'ccr_w/1','ccr_w/1','ccr_w/1','ccr_v/1','ccr_v/1','ccr_w/1'}, [140 335 420 405]);
outs = {'trusted_pair','trusted_phase_a','trusted_phase_b','reconstruct_phase','ccr_x','ccr_y'};
sels = {'trusted_pair_sel','phase_a_sel','phase_b_sel','reconstruct_sel','ccr_x_sel','ccr_y_sel'};
for k = 1:numel(sels)
    ln(s, 'sector/1', [sels{k} '/1']);
    ln(s, [sels{k} '/1'], [outs{k} '/1']);
end
end

function build_sampling_window(s)
clear_subsystem(s);
inp(s, 'ccr_x', 1, [25 55 55 75]);
inp(s, 'ccr_y', 2, [25 125 55 145]);
outp(s, 'win_start', 1, [800 45 830 65]);
outp(s, 'win_end', 2, [800 100 830 120]);
outp(s, 'win_width', 3, [800 155 830 175]);
outp(s, 'adc_sample_ccr', 4, [800 210 830 230]);
outp(s, 'fallback_flag', 5, [800 265 830 285]);
outp(s, 'win1', 6, [800 320 830 340]);
outp(s, 'win2', 7, [800 375 830 395]);
blk(s, 'pair_max', 'simulink/Math Operations/MinMax', [110 75 160 125], 'Function', 'max', 'Inputs', '2');
blk(s, 'margin_const', 'simulink/Sources/Constant', [110 170 180 200], 'Value', 'adc_margin_ccr');
sumblk(s, 'safe_start_raw', '++', [220 105 250 155]);
blk(s, 'arr_minus_margin', 'simulink/Sources/Constant', [220 215 320 245], 'Value', 'pwm_arr - adc_margin_ccr');
blk(s, 'too_late_cmp', 'simulink/Logic and Bit Operations/Relational Operator', [350 170 400 210], 'Operator', '>');
blk(s, 'pwm_arr_const', 'simulink/Sources/Constant', [350 235 420 265], 'Value', 'pwm_arr');
blk(s, 'safe_start_switch', 'simulink/Signal Routing/Switch', [455 140 515 200], 'Criteria', 'u2 ~= 0');
blk(s, 'min_sample_const', 'simulink/Sources/Constant', [455 230 540 260], 'Value', 'adc_sample_min_ccr');
blk(s, 'safe_start_max', 'simulink/Math Operations/MinMax', [575 165 625 215], 'Function', 'max', 'Inputs', '2');
blk(s, 'safe_end_const', 'simulink/Sources/Constant', [575 95 660 125], 'Value', 'adc_sample_max_ccr');
sumblk(s, 'width_raw', '+-', [575 285 605 335]);
blk(s, 'width_sat', 'simulink/Discontinuities/Saturation', [650 290 710 325], 'LowerLimit', '0', 'UpperLimit', 'pwm_arr');
blk(s, 'width_ok_cmp', 'simulink/Logic and Bit Operations/Relational Operator', [650 360 700 400], 'Operator', '>=');
blk(s, 'min_window_const', 'simulink/Sources/Constant', [575 390 660 420], 'Value', 'adc_min_window_ccr');
sumblk(s, 'center_minus_start', '+-', [650 455 680 505]);
sumblk(s, 'end_minus_center', '+-', [650 525 680 575]);
blk(s, 'half_min_window_const', 'simulink/Sources/Constant', [575 600 675 630], 'Value', 'adc_min_window_ccr/2');
blk(s, 'center_const_a', 'simulink/Sources/Constant', [575 450 625 480], 'Value', 'pwm_half_arr');
blk(s, 'center_const_b', 'simulink/Sources/Constant', [575 520 625 550], 'Value', 'pwm_half_arr');
blk(s, 'center_left_ok', 'simulink/Logic and Bit Operations/Relational Operator', [715 455 765 495], 'Operator', '>=');
blk(s, 'center_right_ok', 'simulink/Logic and Bit Operations/Relational Operator', [715 525 765 565], 'Operator', '>=');
blk(s, 'center_safe_and', 'simulink/Logic and Bit Operations/Logical Operator', [805 485 855 545], 'Operator', 'AND', 'Inputs', '2');
gain(s, 'width_half', '0.5', [650 655 710 685]);
sumblk(s, 'mid_raw', '++', [750 635 780 685]);
mathblk(s, 'mid_floor', 'floor', [815 640 875 670]);
blk(s, 'center_const_sample', 'simulink/Sources/Constant', [815 710 875 740], 'Value', 'pwm_half_arr');
blk(s, 'center_or_mid_switch', 'simulink/Signal Routing/Switch', [915 660 975 720], 'Criteria', 'u2 ~= 0');
blk(s, 'fallback_const', 'simulink/Sources/Constant', [915 765 995 795], 'Value', 'adc_fallback_ccr');
blk(s, 'sample_switch', 'simulink/Signal Routing/Switch', [1030 700 1090 760], 'Criteria', 'u2 ~= 0');
blk(s, 'sample_sat', 'simulink/Discontinuities/Saturation', [1125 715 1190 745], 'LowerLimit', 'svpwm_min_ccr', 'UpperLimit', 'svpwm_max_ccr');
blk(s, 'not_width_ok', 'simulink/Logic and Bit Operations/Logical Operator', [745 360 795 400], 'Operator', 'NOT');
blk(s, 'fallback_double', 'simulink/Signal Attributes/Data Type Conversion', [825 365 885 395], 'OutDataTypeStr', 'double');
sumblk(s, 'win1_raw', '+-', [1125 790 1155 840]);
blk(s, 'win1_sat', 'simulink/Discontinuities/Saturation', [1190 795 1255 825], 'LowerLimit', '0', 'UpperLimit', 'pwm_arr');
sumblk(s, 'win2_raw', '+-', [1125 865 1155 915]);
blk(s, 'win2_sat', 'simulink/Discontinuities/Saturation', [1190 870 1255 900], 'LowerLimit', '0', 'UpperLimit', 'pwm_arr');
ln(s, 'ccr_x/1', 'pair_max/1'); ln(s, 'ccr_y/1', 'pair_max/2');
ln(s, 'pair_max/1', 'safe_start_raw/1'); ln(s, 'margin_const/1', 'safe_start_raw/2');
ln(s, 'pair_max/1', 'too_late_cmp/1'); ln(s, 'arr_minus_margin/1', 'too_late_cmp/2');
ln(s, 'pwm_arr_const/1', 'safe_start_switch/1'); ln(s, 'too_late_cmp/1', 'safe_start_switch/2'); ln(s, 'safe_start_raw/1', 'safe_start_switch/3');
ln(s, 'safe_start_switch/1', 'safe_start_max/1'); ln(s, 'min_sample_const/1', 'safe_start_max/2');
ln(s, 'safe_start_max/1', 'win_start/1'); ln(s, 'safe_end_const/1', 'win_end/1');
ln(s, 'safe_end_const/1', 'width_raw/1'); ln(s, 'safe_start_max/1', 'width_raw/2'); ln(s, 'width_raw/1', 'width_sat/1'); ln(s, 'width_sat/1', 'win_width/1');
ln(s, 'width_sat/1', 'width_ok_cmp/1'); ln(s, 'min_window_const/1', 'width_ok_cmp/2');
ln(s, 'width_ok_cmp/1', 'not_width_ok/1'); ln(s, 'not_width_ok/1', 'fallback_double/1'); ln(s, 'fallback_double/1', 'fallback_flag/1');
ln(s, 'center_const_a/1', 'center_minus_start/1'); ln(s, 'safe_start_max/1', 'center_minus_start/2');
ln(s, 'safe_end_const/1', 'end_minus_center/1'); ln(s, 'center_const_b/1', 'end_minus_center/2');
ln(s, 'center_minus_start/1', 'center_left_ok/1'); ln(s, 'half_min_window_const/1', 'center_left_ok/2');
ln(s, 'end_minus_center/1', 'center_right_ok/1'); ln(s, 'half_min_window_const/1', 'center_right_ok/2');
ln(s, 'center_left_ok/1', 'center_safe_and/1'); ln(s, 'center_right_ok/1', 'center_safe_and/2');
ln(s, 'width_sat/1', 'width_half/1'); ln(s, 'safe_start_max/1', 'mid_raw/1'); ln(s, 'width_half/1', 'mid_raw/2'); ln(s, 'mid_raw/1', 'mid_floor/1');
ln(s, 'center_const_sample/1', 'center_or_mid_switch/1'); ln(s, 'center_safe_and/1', 'center_or_mid_switch/2'); ln(s, 'mid_floor/1', 'center_or_mid_switch/3');
ln(s, 'center_or_mid_switch/1', 'sample_switch/1'); ln(s, 'width_ok_cmp/1', 'sample_switch/2'); ln(s, 'fallback_const/1', 'sample_switch/3');
ln(s, 'sample_switch/1', 'sample_sat/1'); ln(s, 'sample_sat/1', 'adc_sample_ccr/1');
ln(s, 'sample_sat/1', 'win1_raw/1'); ln(s, 'safe_start_max/1', 'win1_raw/2'); ln(s, 'win1_raw/1', 'win1_sat/1'); ln(s, 'win1_sat/1', 'win1/1');
ln(s, 'safe_end_const/1', 'win2_raw/1'); ln(s, 'sample_sat/1', 'win2_raw/2'); ln(s, 'win2_raw/1', 'win2_sat/1'); ln(s, 'win2_sat/1', 'win2/1');
end

function build_diagnostics(s)
clear_subsystem(s);
inp(s, 'fallback_flag', 1, [25 55 55 75]);
inp(s, 'win_width', 2, [25 130 55 150]);
outp(s, 'valid_sample_flag', 1, [330 55 360 75]);
outp(s, 'hold_last_flag', 2, [330 130 360 150]);
blk(s, 'not_fallback', 'simulink/Logic and Bit Operations/Logical Operator', [110 50 160 85], 'Operator', 'NOT');
blk(s, 'valid_double', 'simulink/Signal Attributes/Data Type Conversion', [205 55 265 85], 'OutDataTypeStr', 'double');
blk(s, 'hold_enable_const', 'simulink/Sources/Constant', [100 150 170 180], 'Value', 'adc_hold_last_on_fallback');
blk(s, 'hold_and', 'simulink/Logic and Bit Operations/Logical Operator', [205 130 255 180], 'Operator', 'AND', 'Inputs', '2');
blk(s, 'hold_double', 'simulink/Signal Attributes/Data Type Conversion', [285 135 345 165], 'OutDataTypeStr', 'double');
ln(s, 'fallback_flag/1', 'not_fallback/1'); ln(s, 'not_fallback/1', 'valid_double/1'); ln(s, 'valid_double/1', 'valid_sample_flag/1');
ln(s, 'fallback_flag/1', 'hold_and/1'); ln(s, 'hold_enable_const/1', 'hold_and/2'); ln(s, 'hold_and/1', 'hold_double/1'); ln(s, 'hold_double/1', 'hold_last_flag/1');
end

function build_logging(s)
clear_subsystem(s);
for k = 1:20
    inp(s, sprintf('sig_%02d', k), k, [25 20+25*k 55 40+25*k]);
end
blk(s, 'sampling_timing_mux', 'simulink/Signal Routing/Mux', [145 45 185 540], 'Inputs', '20');
blk(s, 'pwm_adc_sampling_log', 'simulink/Sinks/To Workspace', [250 250 400 285], 'VariableName', 'pwm_adc_sampling_log', 'SaveFormat', 'Array');
blk(s, 'SamplingTimingScope', 'simulink/Sinks/Scope', [250 340 400 380]);
for k = 1:20
    ln(s, sprintf('sig_%02d/1', k), sprintf('sampling_timing_mux/%d', k));
end
ln(s, 'sampling_timing_mux/1', 'pwm_adc_sampling_log/1');
ln(s, 'sampling_timing_mux/1', 'SamplingTimingScope/1');
end

function const_select6(s, name, values, pos)
select6(s, name, pos);
for k = 1:6
    blk(s, sprintf('%s_c%d', name, k), 'simulink/Sources/Constant', [pos(1)-75 pos(2)+20*k pos(1)-25 pos(2)+20*k+20], 'Value', values{k});
    ln(s, sprintf('%s_c%d/1', name, k), sprintf('%s/%d', name, k+1));
end
end

function signal_select6(s, name, srcs, pos)
select6(s, name, pos);
for k = 1:6
    ln(s, srcs{k}, sprintf('%s/%d', name, k+1));
end
end

function clear_subsystem(s)
Simulink.SubSystem.deleteContents(s);
end

function inp(s, name, port, pos)
blk(s, name, 'simulink/Sources/In1', pos, 'Port', num2str(port));
end

function outp(s, name, port, pos)
blk(s, name, 'simulink/Sinks/Out1', pos, 'Port', num2str(port));
end

function gain(s, name, g, pos)
blk(s, name, 'simulink/Math Operations/Gain', pos, 'Gain', g);
end

function prod(s, name, pos)
blk(s, name, 'simulink/Math Operations/Product', pos);
end

function sumblk(s, name, inputs, pos)
blk(s, name, 'simulink/Math Operations/Sum', pos, 'Inputs', inputs);
end

function trig(s, name, op, pos)
blk(s, name, 'simulink/Math Operations/Trigonometric Function', pos, 'Operator', op);
end

function mathblk(s, name, op, pos)
if strcmp(op, 'floor')
    blk(s, name, 'simulink/Math Operations/Rounding Function', pos, 'Operator', 'floor');
else
    blk(s, name, 'simulink/Math Operations/Math Function', pos, 'Operator', op);
end
end

function blk(s, name, lib, pos, varargin)
add_block(lib, [s '/' name], 'Position', pos, varargin{:});
end

function ln(s, src, dst)
add_line(s, src, dst, 'autorouting', 'on');
end
