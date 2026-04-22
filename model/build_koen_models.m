function build_koen_models()
%BUILD_KOEN_MODELS Auto-create two white-box Simulink models for this FOC project.
%
% Generated models:
%   1. koen_pmsm_current_loop_model
%   2. koen_flux_observer_pll_model
%
% The script uses only basic Simulink blocks and builds each functional
% part as a subsystem so we can inspect every equation directly.

%% Unified parameters
common.Ts = 6.25e-5;

current.Rs = 0.53;
current.Ld = 0.0006;
current.Lq = 0.0006;
current.psi_f = 0.0078;
current.pole_pairs = 4;
current.Vdc = 24;
current.Vmax = 0.5 * current.Vdc;
current.Kp_id = 2;
current.Ki_id = 100;
current.Kp_iq = 2;
current.Ki_iq = 100;
current.vd_cmd_sign = -1;
current.vq_cmd_sign = -1;
current.wm = 2 * pi * 15;   % 15 Hz mechanical speed, aligned with current code.
current.id_ref = 0;
current.iq_ref = 1;
current.TL = 0;
current.stop_time = 0.05;

observer.Rs = 0.53;
observer.Kp_pll = 200;
observer.Ki_pll = 10000;
observer.theta_comp_rad = 2.7239;
observer.omega_real = 2 * pi * 60;  % electrical rad/s stimulus
observer.v_amp = 6.0;
observer.i_amp = 2.0;
observer.stop_time = 0.05;

out_dir = fileparts(mfilename('fullpath'));

build_current_loop_model(out_dir, common, current);
build_observer_model(out_dir, common, observer);

fprintf('Models created in: %s\n', out_dir);
end

function build_current_loop_model(out_dir, common, p)
mdl = 'koen_pmsm_current_loop_model';
mdl_path = fullfile(out_dir, [mdl '.slx']);
reset_model_file(mdl, mdl_path);

new_system(mdl);
open_system(mdl);
set_param(mdl, ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(common.Ts), ...
    'StopTime', num2str(p.stop_time), ...
    'SystemTargetFile', 'grt.tlc');

% Top-level sources
add_block('simulink/Sources/Constant', [mdl '/id_ref'], ...
    'Value', 'id_ref', 'Position', [30 80 90 110]);
add_block('simulink/Sources/Constant', [mdl '/iq_ref'], ...
    'Value', 'iq_ref', 'Position', [30 140 90 170]);
add_block('simulink/Sources/Constant', [mdl '/wm'], ...
    'Value', 'wm', 'Position', [30 200 90 230]);
add_block('simulink/Sources/Constant', [mdl '/Vdc'], ...
    'Value', 'Vdc', 'Position', [30 260 90 290]);
add_block('simulink/Sources/Constant', [mdl '/TL'], ...
    'Value', 'TL', 'Position', [30 320 90 350]);

set_param(mdl, 'PreLoadFcn', sprintf([ ...
    'Ts=%.10g; Rs=%.10g; Ld=%.10g; Lq=%.10g; psi_f=%.10g; pole_pairs=%.10g; ' ...
    'Vdc=%.10g; Vmax=%.10g; Kp_id=%.10g; Ki_id=%.10g; Kp_iq=%.10g; Ki_iq=%.10g; ' ...
    'FOC_VD_CMD_SIGN=%.10g; FOC_VQ_CMD_SIGN=%.10g; ' ...
    'wm=%.10g; id_ref=%.10g; iq_ref=%.10g; TL=%.10g;'], ...
    common.Ts, p.Rs, p.Ld, p.Lq, p.psi_f, p.pole_pairs, ...
    p.Vdc, p.Vmax, p.Kp_id, p.Ki_id, p.Kp_iq, p.Ki_iq, ...
    p.vd_cmd_sign, p.vq_cmd_sign, ...
    p.wm, p.id_ref, p.iq_ref, p.TL));

% Functional subsystems
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/D_Current_PI'], ...
    'Position', [180 60 340 140]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Q_Current_PI'], ...
    'Position', [180 150 340 230]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Voltage_Command_Sign'], ...
    'Position', [380 95 520 225]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Voltage_Vector_Limit'], ...
    'Position', [580 95 770 225]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/PMSM_dq_Plant'], ...
    'Position', [840 80 1080 260]);

annotate_subsystem([mdl '/D_Current_PI'], 'd-axis PI controller: ref - meas -> PI -> vd_cmd');
annotate_subsystem([mdl '/Q_Current_PI'], 'q-axis PI controller: ref - meas -> PI -> vq_cmd');
annotate_subsystem([mdl '/Voltage_Command_Sign'], 'Map code-level sign definition: FOC_VD_CMD_SIGN / FOC_VQ_CMD_SIGN');
annotate_subsystem([mdl '/Voltage_Vector_Limit'], 'dq voltage vector limiter: Vmag and scaling');
annotate_subsystem([mdl '/PMSM_dq_Plant'], 'White-box PMSM dq electrical model using discrete integrators');

populate_pi_subsystem([mdl '/D_Current_PI'], 'Kp_id', 'Ki_id', 'Ts', 'vd_pi');
populate_pi_subsystem([mdl '/Q_Current_PI'], 'Kp_iq', 'Ki_iq', 'Ts', 'vq_pi');
populate_voltage_sign_subsystem([mdl '/Voltage_Command_Sign']);
populate_voltage_limit_subsystem([mdl '/Voltage_Vector_Limit']);
populate_pmsm_plant_subsystem([mdl '/PMSM_dq_Plant']);

% Scopes and outputs
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_id_iq'], ...
    'Inputs', '2', 'Position', [980 80 1005 130]);
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_iq_ref'], ...
    'Inputs', '2', 'Position', [980 150 1005 200]);
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_vd_vq'], ...
    'Inputs', '2', 'Position', [980 220 1005 270]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_id_iq'], ...
    'Position', [1200 70 1330 140]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_iq_ref_vs_iq'], ...
    'Position', [1200 140 1330 210]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_vd_vq'], ...
    'Position', [1200 210 1330 280]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_Vmag'], ...
    'Position', [1200 290 1330 350]);

add_block('simulink/Sinks/Out1', [mdl '/id'], 'Position', [1140 370 1170 390]);
add_block('simulink/Sinks/Out1', [mdl '/iq'], 'Position', [1140 400 1170 420]);
add_block('simulink/Sinks/Out1', [mdl '/vd'], 'Position', [1140 430 1170 450]);
add_block('simulink/Sinks/Out1', [mdl '/vq'], 'Position', [1140 460 1170 480]);
add_block('simulink/Sinks/Out1', [mdl '/V_mag'], 'Position', [1140 490 1170 510]);

% Wiring
safe_add_line(mdl, 'id_ref/1', 'D_Current_PI/1');
safe_add_line(mdl, 'iq_ref/1', 'Q_Current_PI/1');
safe_add_line(mdl, 'PMSM_dq_Plant/1', 'D_Current_PI/2');
safe_add_line(mdl, 'PMSM_dq_Plant/2', 'Q_Current_PI/2');
safe_add_line(mdl, 'D_Current_PI/1', 'Voltage_Command_Sign/1');
safe_add_line(mdl, 'Q_Current_PI/1', 'Voltage_Command_Sign/2');
safe_add_line(mdl, 'Voltage_Command_Sign/1', 'Voltage_Vector_Limit/1');
safe_add_line(mdl, 'Voltage_Command_Sign/2', 'Voltage_Vector_Limit/2');
safe_add_line(mdl, 'Vdc/1', 'Voltage_Vector_Limit/3');
safe_add_line(mdl, 'Voltage_Vector_Limit/1', 'PMSM_dq_Plant/1');
safe_add_line(mdl, 'Voltage_Vector_Limit/2', 'PMSM_dq_Plant/2');
safe_add_line(mdl, 'wm/1', 'PMSM_dq_Plant/3');
safe_add_line(mdl, 'TL/1', 'PMSM_dq_Plant/4');

safe_add_line(mdl, 'PMSM_dq_Plant/1', 'Mux_id_iq/1');
safe_add_line(mdl, 'PMSM_dq_Plant/2', 'Mux_id_iq/2');
safe_add_line(mdl, 'Mux_id_iq/1', 'Scope_id_iq/1');

safe_add_line(mdl, 'iq_ref/1', 'Mux_iq_ref/1');
safe_add_line(mdl, 'PMSM_dq_Plant/2', 'Mux_iq_ref/2');
safe_add_line(mdl, 'Mux_iq_ref/1', 'Scope_iq_ref_vs_iq/1');

safe_add_line(mdl, 'Voltage_Vector_Limit/1', 'Mux_vd_vq/1');
safe_add_line(mdl, 'Voltage_Vector_Limit/2', 'Mux_vd_vq/2');
safe_add_line(mdl, 'Mux_vd_vq/1', 'Scope_vd_vq/1');
safe_add_line(mdl, 'Voltage_Vector_Limit/3', 'Scope_Vmag/1');

safe_add_line(mdl, 'PMSM_dq_Plant/1', 'id/1');
safe_add_line(mdl, 'PMSM_dq_Plant/2', 'iq/1');
safe_add_line(mdl, 'Voltage_Vector_Limit/1', 'vd/1');
safe_add_line(mdl, 'Voltage_Vector_Limit/2', 'vq/1');
safe_add_line(mdl, 'Voltage_Vector_Limit/3', 'V_mag/1');

arrange_and_save(mdl, mdl_path);
end

function build_observer_model(out_dir, common, p)
mdl = 'koen_flux_observer_pll_model';
mdl_path = fullfile(out_dir, [mdl '.slx']);
reset_model_file(mdl, mdl_path);

new_system(mdl);
open_system(mdl);
set_param(mdl, ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(common.Ts), ...
    'StopTime', num2str(p.stop_time), ...
    'SystemTargetFile', 'grt.tlc');

set_param(mdl, 'PreLoadFcn', sprintf([ ...
    'Ts=%.10g; Rs=%.10g; Kp_pll=%.10g; Ki_pll=%.10g; FOC_OBS_THETA_COMP_RAD=%.10g; omega_real=%.10g; ' ...
    'v_amp=%.10g; i_amp=%.10g;'], ...
    common.Ts, p.Rs, p.Kp_pll, p.Ki_pll, p.theta_comp_rad, p.omega_real, p.v_amp, p.i_amp));

add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Input_Stimulus'], ...
    'Position', [40 90 220 260]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Flux_Observer'], ...
    'Position', [310 100 500 240]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Theta_Calculation'], ...
    'Position', [580 120 730 210]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/PLL'], ...
    'Position', [820 110 980 240]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Theta_Compensation'], ...
    'Position', [1030 110 1180 200]);

annotate_subsystem([mdl '/Input_Stimulus'], 'White-box rotating alpha-beta test source derived from theta_real');
annotate_subsystem([mdl '/Flux_Observer'], 'Flux observer: integrate Valpha - Rs*Ialpha and Vbeta - Rs*Ibeta');
annotate_subsystem([mdl '/Theta_Calculation'], 'theta_raw = atan2(psi_beta, psi_alpha)');
annotate_subsystem([mdl '/PLL'], 'PLL PI structure: theta_raw -> error -> omega_est -> theta_est');
annotate_subsystem([mdl '/Theta_Compensation'], 'Map code-level observer output offset: FOC_OBS_THETA_COMP_RAD');

populate_input_stimulus_subsystem([mdl '/Input_Stimulus']);
populate_flux_observer_subsystem([mdl '/Flux_Observer']);
populate_theta_calc_subsystem([mdl '/Theta_Calculation']);
populate_pll_subsystem([mdl '/PLL']);
populate_theta_comp_subsystem([mdl '/Theta_Compensation']);

add_block('simulink/Signal Routing/Mux', [mdl '/Mux_theta_compare'], ...
    'Inputs', '2', 'Position', [1020 110 1045 160]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_theta_compare'], ...
    'Position', [1260 100 1390 170]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_theta_error'], ...
    'Position', [1260 190 1390 250]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_omega_est'], ...
    'Position', [1260 280 1390 340]);

add_block('simulink/Sinks/Out1', [mdl '/theta_raw'], 'Position', [1210 380 1240 400]);
add_block('simulink/Sinks/Out1', [mdl '/theta_est'], 'Position', [1210 410 1240 430]);
add_block('simulink/Sinks/Out1', [mdl '/omega_est'], 'Position', [1210 440 1240 460]);
add_block('simulink/Sinks/Out1', [mdl '/theta_error'], 'Position', [1210 470 1240 490]);

safe_add_line(mdl, 'Input_Stimulus/1', 'Flux_Observer/1');
safe_add_line(mdl, 'Input_Stimulus/2', 'Flux_Observer/2');
safe_add_line(mdl, 'Input_Stimulus/3', 'Flux_Observer/3');
safe_add_line(mdl, 'Input_Stimulus/4', 'Flux_Observer/4');
safe_add_line(mdl, 'Flux_Observer/1', 'Theta_Calculation/1');
safe_add_line(mdl, 'Flux_Observer/2', 'Theta_Calculation/2');
safe_add_line(mdl, 'Theta_Calculation/1', 'PLL/1');
safe_add_line(mdl, 'PLL/1', 'Theta_Compensation/1');

safe_add_line(mdl, 'Theta_Calculation/1', 'Mux_theta_compare/1');
safe_add_line(mdl, 'Theta_Compensation/1', 'Mux_theta_compare/2');
safe_add_line(mdl, 'Mux_theta_compare/1', 'Scope_theta_compare/1');
safe_add_line(mdl, 'Theta_Compensation/2', 'Scope_theta_error/1');
safe_add_line(mdl, 'PLL/2', 'Scope_omega_est/1');

safe_add_line(mdl, 'Theta_Calculation/1', 'theta_raw/1');
safe_add_line(mdl, 'Theta_Compensation/1', 'theta_est/1');
safe_add_line(mdl, 'PLL/2', 'omega_est/1');
safe_add_line(mdl, 'Theta_Compensation/2', 'theta_error/1');

arrange_and_save(mdl, mdl_path);
end

function populate_pi_subsystem(subsys, kp_name, ki_name, ts_name, output_name)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/ref'], 'Position', [25 48 55 62]);
add_block('simulink/Sources/In1', [subsys '/meas'], 'Position', [25 108 55 122]);
add_block('simulink/Math Operations/Sum', [subsys '/Error'], ...
    'Inputs', '+-', 'IconShape', 'round', 'Position', [85 72 115 98]);
add_block('simulink/Math Operations/Gain', [subsys '/Kp'], ...
    'Gain', kp_name, 'Position', [150 48 220 72]);
add_block('simulink/Math Operations/Gain', [subsys '/Ki'], ...
    'Gain', ki_name, 'Position', [150 108 220 132]);
add_block('simulink/Discrete/Discrete-Time Integrator', [subsys '/Integrator'], ...
    'SampleTime', ts_name, 'InitialCondition', '0', 'Position', [255 100 295 140]);
add_block('simulink/Discontinuities/Saturation', [subsys '/I_Limit'], ...
    'UpperLimit', '20', 'LowerLimit', '-20', 'Position', [330 100 380 140]);
add_block('simulink/Math Operations/Sum', [subsys '/PI_Sum'], ...
    'Inputs', '++', 'IconShape', 'round', 'Position', [330 55 360 85]);
add_block('simulink/Discontinuities/Saturation', [subsys '/Output_Limit'], ...
    'UpperLimit', '15', 'LowerLimit', '-15', 'Position', [405 55 455 85]);
add_block('simulink/Sinks/Out1', [subsys '/' output_name], ...
    'Position', [500 63 530 77]);

safe_add_line(subsys, 'ref/1', 'Error/1');
safe_add_line(subsys, 'meas/1', 'Error/2');
safe_add_line(subsys, 'Error/1', 'Kp/1');
safe_add_line(subsys, 'Error/1', 'Ki/1');
safe_add_line(subsys, 'Ki/1', 'Integrator/1');
safe_add_line(subsys, 'Integrator/1', 'I_Limit/1');
safe_add_line(subsys, 'Kp/1', 'PI_Sum/1');
safe_add_line(subsys, 'I_Limit/1', 'PI_Sum/2');
safe_add_line(subsys, 'PI_Sum/1', 'Output_Limit/1');
safe_add_line(subsys, 'Output_Limit/1', [output_name '/1']);

add_annotation(subsys, [20 10 300 35], 'Separate P and I paths with discrete integrator and output saturation.');
arrange_subsystem(subsys);
end

function populate_voltage_limit_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/vd_in'], 'Position', [25 48 55 62]);
add_block('simulink/Sources/In1', [subsys '/vq_in'], 'Position', [25 108 55 122]);
add_block('simulink/Sources/In1', [subsys '/Vdc'], 'Position', [25 168 55 182]);

add_block('simulink/Math Operations/Product', [subsys '/vd_sq'], 'Position', [95 38 135 72]);
add_block('simulink/Math Operations/Product', [subsys '/vq_sq'], 'Position', [95 98 135 132]);
add_block('simulink/Math Operations/Sum', [subsys '/mag_sq'], ...
    'Inputs', '++', 'Position', [165 70 195 100]);
add_block('simulink/Math Operations/Math Function', [subsys '/sqrt'], ...
    'Operator', 'sqrt', 'Position', [225 68 265 102]);
add_block('simulink/Math Operations/Gain', [subsys '/Vmax'], ...
    'Gain', '0.5', 'Position', [225 158 285 182]);
add_block('simulink/Sources/Constant', [subsys '/eps'], ...
    'Value', '1e-6', 'Position', [225 115 255 135]);
add_block('simulink/Math Operations/Sum', [subsys '/mag_safe'], ...
    'Inputs', '++', 'Position', [285 82 315 108]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/mag_gt_vmax'], ...
    'Operator', '>', 'Position', [335 148 385 182]);
add_block('simulink/Math Operations/Divide', [subsys '/scale_raw'], ...
    'Position', [345 78 385 112]);
add_block('simulink/Sources/Constant', [subsys '/one'], ...
    'Value', '1', 'Position', [410 120 440 140]);
add_block('simulink/Signal Routing/Switch', [subsys '/Switch'], ...
    'Threshold', '0.5', 'Criteria', 'u2 ~= 0', 'Position', [460 83 500 147]);
add_block('simulink/Math Operations/Product', [subsys '/vd_out_calc'], ...
    'Position', [530 42 570 76]);
add_block('simulink/Math Operations/Product', [subsys '/vq_out_calc'], ...
    'Position', [530 102 570 136]);
add_block('simulink/Sinks/Out1', [subsys '/vd_out'], 'Position', [620 50 650 64]);
add_block('simulink/Sinks/Out1', [subsys '/vq_out'], 'Position', [620 110 650 124]);
add_block('simulink/Sinks/Out1', [subsys '/V_mag'], 'Position', [620 170 650 184]);

safe_add_line(subsys, 'vd_in/1', 'vd_sq/1');
safe_add_line(subsys, 'vd_in/1', 'vd_sq/2');
safe_add_line(subsys, 'vq_in/1', 'vq_sq/1');
safe_add_line(subsys, 'vq_in/1', 'vq_sq/2');
safe_add_line(subsys, 'vd_sq/1', 'mag_sq/1');
safe_add_line(subsys, 'vq_sq/1', 'mag_sq/2');
safe_add_line(subsys, 'mag_sq/1', 'sqrt/1');
safe_add_line(subsys, 'sqrt/1', 'mag_safe/1');
safe_add_line(subsys, 'eps/1', 'mag_safe/2');
safe_add_line(subsys, 'Vdc/1', 'Vmax/1');
safe_add_line(subsys, 'sqrt/1', 'V_mag/1');
safe_add_line(subsys, 'Vmax/1', 'mag_gt_vmax/1');
safe_add_line(subsys, 'sqrt/1', 'mag_gt_vmax/2');
safe_add_line(subsys, 'Vmax/1', 'scale_raw/1');
safe_add_line(subsys, 'mag_safe/1', 'scale_raw/2');
safe_add_line(subsys, 'scale_raw/1', 'Switch/1');
safe_add_line(subsys, 'mag_gt_vmax/1', 'Switch/2');
safe_add_line(subsys, 'one/1', 'Switch/3');
safe_add_line(subsys, 'vd_in/1', 'vd_out_calc/1');
safe_add_line(subsys, 'vq_in/1', 'vq_out_calc/1');
safe_add_line(subsys, 'Switch/1', 'vd_out_calc/2');
safe_add_line(subsys, 'Switch/1', 'vq_out_calc/2');
safe_add_line(subsys, 'vd_out_calc/1', 'vd_out/1');
safe_add_line(subsys, 'vq_out_calc/1', 'vq_out/1');

add_annotation(subsys, [20 10 360 35], 'Scale vd and vq when sqrt(vd^2 + vq^2) exceeds Vmax = 0.5 * Vdc.');
arrange_subsystem(subsys);
end

function populate_voltage_sign_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/vd_in'], 'Position', [25 48 55 62]);
add_block('simulink/Sources/In1', [subsys '/vq_in'], 'Position', [25 108 55 122]);
add_block('simulink/Math Operations/Gain', [subsys '/vd_sign'], ...
    'Gain', 'FOC_VD_CMD_SIGN', 'Position', [110 42 190 68]);
add_block('simulink/Math Operations/Gain', [subsys '/vq_sign'], ...
    'Gain', 'FOC_VQ_CMD_SIGN', 'Position', [110 102 190 128]);
add_block('simulink/Sinks/Out1', [subsys '/vd_out'], 'Position', [245 50 275 64]);
add_block('simulink/Sinks/Out1', [subsys '/vq_out'], 'Position', [245 110 275 124]);

safe_add_line(subsys, 'vd_in/1', 'vd_sign/1');
safe_add_line(subsys, 'vq_in/1', 'vq_sign/1');
safe_add_line(subsys, 'vd_sign/1', 'vd_out/1');
safe_add_line(subsys, 'vq_sign/1', 'vq_out/1');

add_annotation(subsys, [20 10 260 30], 'Apply the same vd/vq polarity mapping used by the firmware current loop.');
arrange_subsystem(subsys);
end

function populate_pmsm_plant_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/vd'], 'Position', [25 58 55 72]);
add_block('simulink/Sources/In1', [subsys '/vq'], 'Position', [25 118 55 132]);
add_block('simulink/Sources/In1', [subsys '/wm'], 'Position', [25 178 55 192]);
add_block('simulink/Sources/In1', [subsys '/TL'], 'Position', [25 238 55 252]);
add_block('simulink/Sinks/Terminator', [subsys '/TL_unused'], 'Position', [100 235 120 255]);

add_block('simulink/Math Operations/Gain', [subsys '/pole_pairs'], ...
    'Gain', 'pole_pairs', 'Position', [95 175 165 195]);
add_block('simulink/Sinks/Out1', [subsys '/we'], 'Position', [650 185 680 199]);

add_block('simulink/Discrete/Discrete-Time Integrator', [subsys '/id_int'], ...
    'SampleTime', 'Ts', 'InitialCondition', '0', 'Position', [585 45 625 85]);
add_block('simulink/Discrete/Discrete-Time Integrator', [subsys '/iq_int'], ...
    'SampleTime', 'Ts', 'InitialCondition', '0', 'Position', [585 105 625 145]);
add_block('simulink/Sinks/Out1', [subsys '/id'], 'Position', [650 55 680 69]);
add_block('simulink/Sinks/Out1', [subsys '/iq'], 'Position', [650 115 680 129]);

% did/dt path
add_block('simulink/Math Operations/Gain', [subsys '/Rs_id'], ...
    'Gain', 'Rs', 'Position', [225 35 285 55]);
add_block('simulink/Math Operations/Product', [subsys '/we_iq'], ...
    'Position', [225 80 265 110]);
add_block('simulink/Math Operations/Gain', [subsys '/Lq_gain'], ...
    'Gain', 'Lq', 'Position', [295 85 355 105]);
add_block('simulink/Math Operations/Sum', [subsys '/did_num'], ...
    'Inputs', '+-+', 'Position', [395 58 425 92]);
add_block('simulink/Math Operations/Gain', [subsys '/inv_Ld'], ...
    'Gain', '1/Ld', 'Position', [465 65 525 85]);

% diq/dt path
add_block('simulink/Math Operations/Gain', [subsys '/Rs_iq'], ...
    'Gain', 'Rs', 'Position', [225 135 285 155]);
add_block('simulink/Math Operations/Gain', [subsys '/Ld_id'], ...
    'Gain', 'Ld', 'Position', [225 185 285 205]);
add_block('simulink/Sources/Constant', [subsys '/psi_f'], ...
    'Value', 'psi_f', 'Position', [305 215 345 235]);
add_block('simulink/Math Operations/Sum', [subsys '/flux_sum'], ...
    'Inputs', '++', 'Position', [375 188 405 222]);
add_block('simulink/Math Operations/Product', [subsys '/we_flux'], ...
    'Position', [435 185 475 215]);
add_block('simulink/Math Operations/Sum', [subsys '/diq_num'], ...
    'Inputs', '+--', 'Position', [505 118 535 152]);
add_block('simulink/Math Operations/Gain', [subsys '/inv_Lq'], ...
    'Gain', '1/Lq', 'Position', [545 125 575 145]);

safe_add_line(subsys, 'TL/1', 'TL_unused/1');
safe_add_line(subsys, 'wm/1', 'pole_pairs/1');
safe_add_line(subsys, 'pole_pairs/1', 'we/1');

safe_add_line(subsys, 'id_int/1', 'id/1');
safe_add_line(subsys, 'iq_int/1', 'iq/1');

safe_add_line(subsys, 'id_int/1', 'Rs_id/1');
safe_add_line(subsys, 'pole_pairs/1', 'we_iq/1');
safe_add_line(subsys, 'iq_int/1', 'we_iq/2');
safe_add_line(subsys, 'we_iq/1', 'Lq_gain/1');
safe_add_line(subsys, 'vd/1', 'did_num/1');
safe_add_line(subsys, 'Rs_id/1', 'did_num/2');
safe_add_line(subsys, 'Lq_gain/1', 'did_num/3');
safe_add_line(subsys, 'did_num/1', 'inv_Ld/1');
safe_add_line(subsys, 'inv_Ld/1', 'id_int/1');

safe_add_line(subsys, 'iq_int/1', 'Rs_iq/1');
safe_add_line(subsys, 'id_int/1', 'Ld_id/1');
safe_add_line(subsys, 'Ld_id/1', 'flux_sum/1');
safe_add_line(subsys, 'psi_f/1', 'flux_sum/2');
safe_add_line(subsys, 'pole_pairs/1', 'we_flux/1');
safe_add_line(subsys, 'flux_sum/1', 'we_flux/2');
safe_add_line(subsys, 'vq/1', 'diq_num/1');
safe_add_line(subsys, 'Rs_iq/1', 'diq_num/2');
safe_add_line(subsys, 'we_flux/1', 'diq_num/3');
safe_add_line(subsys, 'diq_num/1', 'inv_Lq/1');
safe_add_line(subsys, 'inv_Lq/1', 'iq_int/1');

add_annotation(subsys, [20 10 420 35], ...
    'Electrical dq model only: TL is reserved for future mechanical extension and is terminated here.');
arrange_subsystem(subsys);
end

function populate_input_stimulus_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/Constant', [subsys '/omega_real'], ...
    'Value', 'omega_real', 'Position', [25 40 95 60]);
add_block('simulink/Discrete/Discrete-Time Integrator', [subsys '/theta_real_int'], ...
    'SampleTime', 'Ts', 'InitialCondition', '0', 'Position', [135 35 175 65]);
add_block('simulink/Math Operations/Trigonometric Function', [subsys '/cos'], ...
    'Operator', 'cos', 'Position', [220 25 270 55]);
add_block('simulink/Math Operations/Trigonometric Function', [subsys '/sin'], ...
    'Operator', 'sin', 'Position', [220 75 270 105]);
add_block('simulink/Sources/Constant', [subsys '/v_amp'], ...
    'Value', 'v_amp', 'Position', [305 20 365 40]);
add_block('simulink/Sources/Constant', [subsys '/i_amp'], ...
    'Value', 'i_amp', 'Position', [305 80 365 100]);
add_block('simulink/Math Operations/Product', [subsys '/Valpha_calc'], ...
    'Position', [395 20 435 50]);
add_block('simulink/Math Operations/Product', [subsys '/Vbeta_calc'], ...
    'Position', [395 60 435 90]);
add_block('simulink/Math Operations/Product', [subsys '/Ialpha_calc'], ...
    'Position', [395 100 435 130]);
add_block('simulink/Math Operations/Product', [subsys '/Ibeta_calc'], ...
    'Position', [395 140 435 170]);
add_block('simulink/Sinks/Out1', [subsys '/Valpha'], 'Position', [485 30 515 44]);
add_block('simulink/Sinks/Out1', [subsys '/Vbeta'], 'Position', [485 70 515 84]);
add_block('simulink/Sinks/Out1', [subsys '/Ialpha'], 'Position', [485 110 515 124]);
add_block('simulink/Sinks/Out1', [subsys '/Ibeta'], 'Position', [485 150 515 164]);
add_block('simulink/Sinks/Out1', [subsys '/theta_real'], 'Position', [485 190 515 204]);

safe_add_line(subsys, 'omega_real/1', 'theta_real_int/1');
safe_add_line(subsys, 'theta_real_int/1', 'cos/1');
safe_add_line(subsys, 'theta_real_int/1', 'sin/1');
safe_add_line(subsys, 'theta_real_int/1', 'theta_real/1');
safe_add_line(subsys, 'v_amp/1', 'Valpha_calc/1');
safe_add_line(subsys, 'cos/1', 'Valpha_calc/2');
safe_add_line(subsys, 'v_amp/1', 'Vbeta_calc/1');
safe_add_line(subsys, 'sin/1', 'Vbeta_calc/2');
safe_add_line(subsys, 'i_amp/1', 'Ialpha_calc/1');
safe_add_line(subsys, 'cos/1', 'Ialpha_calc/2');
safe_add_line(subsys, 'i_amp/1', 'Ibeta_calc/1');
safe_add_line(subsys, 'sin/1', 'Ibeta_calc/2');
safe_add_line(subsys, 'Valpha_calc/1', 'Valpha/1');
safe_add_line(subsys, 'Vbeta_calc/1', 'Vbeta/1');
safe_add_line(subsys, 'Ialpha_calc/1', 'Ialpha/1');
safe_add_line(subsys, 'Ibeta_calc/1', 'Ibeta/1');

add_annotation(subsys, [20 5 330 25], 'Generate rotating alpha-beta voltages and currents from theta_real.');
arrange_subsystem(subsys);
end

function populate_flux_observer_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/Valpha'], 'Position', [25 40 55 54]);
add_block('simulink/Sources/In1', [subsys '/Vbeta'], 'Position', [25 90 55 104]);
add_block('simulink/Sources/In1', [subsys '/Ialpha'], 'Position', [25 140 55 154]);
add_block('simulink/Sources/In1', [subsys '/Ibeta'], 'Position', [25 190 55 204]);
add_block('simulink/Math Operations/Gain', [subsys '/Rs_Ialpha'], ...
    'Gain', 'Rs', 'Position', [110 130 170 150]);
add_block('simulink/Math Operations/Gain', [subsys '/Rs_Ibeta'], ...
    'Gain', 'Rs', 'Position', [110 180 170 200]);
add_block('simulink/Math Operations/Sum', [subsys '/psi_alpha_dot'], ...
    'Inputs', '+-', 'Position', [210 48 240 82]);
add_block('simulink/Math Operations/Sum', [subsys '/psi_beta_dot'], ...
    'Inputs', '+-', 'Position', [210 98 240 132]);
add_block('simulink/Discrete/Discrete-Time Integrator', [subsys '/psi_alpha_int'], ...
    'SampleTime', 'Ts', 'InitialCondition', '0', 'Position', [285 45 325 85]);
add_block('simulink/Discrete/Discrete-Time Integrator', [subsys '/psi_beta_int'], ...
    'SampleTime', 'Ts', 'InitialCondition', '0', 'Position', [285 95 325 135]);
add_block('simulink/Sinks/Out1', [subsys '/psi_alpha'], 'Position', [380 55 410 69]);
add_block('simulink/Sinks/Out1', [subsys '/psi_beta'], 'Position', [380 105 410 119]);

safe_add_line(subsys, 'Ialpha/1', 'Rs_Ialpha/1');
safe_add_line(subsys, 'Ibeta/1', 'Rs_Ibeta/1');
safe_add_line(subsys, 'Valpha/1', 'psi_alpha_dot/1');
safe_add_line(subsys, 'Rs_Ialpha/1', 'psi_alpha_dot/2');
safe_add_line(subsys, 'Vbeta/1', 'psi_beta_dot/1');
safe_add_line(subsys, 'Rs_Ibeta/1', 'psi_beta_dot/2');
safe_add_line(subsys, 'psi_alpha_dot/1', 'psi_alpha_int/1');
safe_add_line(subsys, 'psi_beta_dot/1', 'psi_beta_int/1');
safe_add_line(subsys, 'psi_alpha_int/1', 'psi_alpha/1');
safe_add_line(subsys, 'psi_beta_int/1', 'psi_beta/1');

add_annotation(subsys, [20 5 330 25], 'Integrate psi_dot = V - Rs * I for alpha and beta axes.');
arrange_subsystem(subsys);
end

function populate_theta_calc_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/psi_alpha'], 'Position', [25 48 55 62]);
add_block('simulink/Sources/In1', [subsys '/psi_beta'], 'Position', [25 108 55 122]);
add_block('simulink/Math Operations/Trigonometric Function', [subsys '/atan2'], ...
    'Operator', 'atan2', 'Position', [105 70 155 100]);
add_block('simulink/Sinks/Out1', [subsys '/theta_raw'], 'Position', [220 78 250 92]);

safe_add_line(subsys, 'psi_beta/1', 'atan2/1');
safe_add_line(subsys, 'psi_alpha/1', 'atan2/2');
safe_add_line(subsys, 'atan2/1', 'theta_raw/1');

add_annotation(subsys, [20 10 230 30], 'theta_raw = atan2(psi_beta, psi_alpha).');
arrange_subsystem(subsys);
end

function populate_pll_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/theta_raw'], 'Position', [25 78 55 92]);
add_block('simulink/Discrete/Discrete-Time Integrator', [subsys '/theta_est_int'], ...
    'SampleTime', 'Ts', 'InitialCondition', '0', 'Position', [395 115 435 155]);
add_block('simulink/Math Operations/Sum', [subsys '/error_sum'], ...
    'Inputs', '+-', 'Position', [110 78 140 112]);
add_block('simulink/Math Operations/Gain', [subsys '/Kp'], ...
    'Gain', 'Kp_pll', 'Position', [180 58 240 78]);
add_block('simulink/Math Operations/Gain', [subsys '/Ki'], ...
    'Gain', 'Ki_pll', 'Position', [180 118 240 138]);
add_block('simulink/Discrete/Discrete-Time Integrator', [subsys '/err_int'], ...
    'SampleTime', 'Ts', 'InitialCondition', '0', 'Position', [275 110 315 150]);
add_block('simulink/Math Operations/Sum', [subsys '/omega_sum'], ...
    'Inputs', '++', 'Position', [345 76 375 110]);
add_block('simulink/Sinks/Out1', [subsys '/theta_est'], 'Position', [500 125 530 139]);
add_block('simulink/Sinks/Out1', [subsys '/omega_est'], 'Position', [500 85 530 99]);
add_block('simulink/Sinks/Out1', [subsys '/theta_error'], 'Position', [500 45 530 59]);

safe_add_line(subsys, 'theta_raw/1', 'error_sum/1');
safe_add_line(subsys, 'theta_est_int/1', 'error_sum/2');
safe_add_line(subsys, 'error_sum/1', 'theta_error/1');
safe_add_line(subsys, 'error_sum/1', 'Kp/1');
safe_add_line(subsys, 'error_sum/1', 'Ki/1');
safe_add_line(subsys, 'Ki/1', 'err_int/1');
safe_add_line(subsys, 'Kp/1', 'omega_sum/1');
safe_add_line(subsys, 'err_int/1', 'omega_sum/2');
safe_add_line(subsys, 'omega_sum/1', 'omega_est/1');
safe_add_line(subsys, 'omega_sum/1', 'theta_est_int/1');
safe_add_line(subsys, 'theta_est_int/1', 'theta_est/1');

add_annotation(subsys, [20 10 320 30], 'error = theta_raw - theta_est, then PI gives omega_est and integrates back to theta_est.');
arrange_subsystem(subsys);
end

function populate_theta_comp_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/theta_est_raw'], 'Position', [25 48 55 62]);
add_block('simulink/Sources/Constant', [subsys '/theta_comp'], ...
    'Value', 'FOC_OBS_THETA_COMP_RAD', 'Position', [25 108 95 128]);
add_block('simulink/Math Operations/Sum', [subsys '/theta_sum'], ...
    'Inputs', '++', 'Position', [135 58 165 92]);
add_block('simulink/Math Operations/Math Function', [subsys '/mod_2pi'], ...
    'Operator', 'mod', 'Position', [205 56 255 94]);
add_block('simulink/Sources/Constant', [subsys '/two_pi'], ...
    'Value', '2*pi', 'Position', [205 108 255 128]);
add_block('simulink/Math Operations/Sum', [subsys '/theta_error'], ...
    'Inputs', '+-', 'Position', [205 150 235 184]);
add_block('simulink/Sinks/Out1', [subsys '/theta_est_used'], 'Position', [295 65 325 79]);
add_block('simulink/Sinks/Out1', [subsys '/theta_comp_error'], 'Position', [295 158 325 172]);

safe_add_line(subsys, 'theta_est_raw/1', 'theta_sum/1');
safe_add_line(subsys, 'theta_comp/1', 'theta_sum/2');
safe_add_line(subsys, 'theta_sum/1', 'mod_2pi/1');
safe_add_line(subsys, 'two_pi/1', 'mod_2pi/2');
safe_add_line(subsys, 'mod_2pi/1', 'theta_est_used/1');
safe_add_line(subsys, 'theta_sum/1', 'theta_error/1');
safe_add_line(subsys, 'theta_est_raw/1', 'theta_error/2');
safe_add_line(subsys, 'theta_error/1', 'theta_comp_error/1');

add_annotation(subsys, [20 10 300 30], 'Mirror firmware post-PLL angle compensation: theta_used = mod(theta_raw + comp, 2*pi).');
arrange_subsystem(subsys);
end

function reset_model_file(mdl, mdl_path)
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

if exist(mdl_path, 'file') == 2
    delete(mdl_path);
end
end

function clear_subsystem(subsys)
blocks = find_system(subsys, 'SearchDepth', 1, 'Type', 'Block');
for k = 1:numel(blocks)
    if strcmp(blocks{k}, subsys)
        continue;
    end
    delete_block(blocks{k});
end

lines = find_system(subsys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
for k = 1:numel(lines)
    delete_line(lines(k));
end
end

function annotate_subsystem(subsys, text_str)
set_param(subsys, 'Description', text_str);
parent = get_param(subsys, 'Parent');
pos = get_param(subsys, 'Position');
x = pos(1);
y = max(5, pos(2) - 25);
w = max(240, pos(3) - pos(1));
h = 18;
add_annotation(parent, [x y x + w y + h], text_str);
end

function add_annotation(sys, position, text_str)
ann = Simulink.Annotation(sys, text_str);
set(ann, 'Position', position);
end

function safe_add_line(sys, src, dst)
add_line(sys, src, dst, 'autorouting', 'on');
end

function arrange_subsystem(subsys)
try
    Simulink.BlockDiagram.arrangeSystem(subsys);
catch
end
end

function arrange_and_save(mdl, mdl_path)
try
    Simulink.BlockDiagram.arrangeSystem(mdl);
catch
end

save_system(mdl, mdl_path);
close_system(mdl);
end
