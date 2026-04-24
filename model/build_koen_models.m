function build_koen_models()
%BUILD_KOEN_MODELS Build focused analysis models for the current project.
%
% Generated models:
%   1. koen_q_axis_current_loop_model
%   2. koen_three_shunt_sampling_model
%   3. koen_observer_pll_model

common.Ts = 6.25e-5;

current.FOC_TS_SEC = 6.25e-5;
current.FOC_IQ_KP = 3.770;
current.FOC_IQ_KI = 3330.0;
current.FOC_CURR_LOOP_V_LIMIT = 13.5;
current.FOC_DQ_VOLT_LIMIT = 13.5;
current.FOC_VQ_CMD_SIGN = -1;
current.FOC_RS_OHM = 0.53;
current.FOC_LQ_H = 0.0006;
current.FOC_FLUX_WB = 0.0078;
current.FOC_PWM_HALF_ARR = 2655;
current.stop_time = 0.02;
current.OMEGA_E_RAD_S = 0.0;

observer.FOC_TS_SEC = 6.25e-5;
observer.FOC_RS_OHM = 0.53;
observer.FOC_LS_H = 0.0006;
observer.FOC_FLUX_WB = 0.0078;
observer.FOC_FLUX_LIMIT_RATIO = 1.50;
observer.FOC_OBS_PLL_KP = 180.0;
observer.FOC_OBS_PLL_KI = 6000.0;
observer.FOC_OBS_PLL_INTEGRAL_LIMIT = 700.0;
observer.FOC_OBS_SPEED_LIMIT_RAD_S = 1200.0;
observer.FOC_OBS_SPEED_FILTER_ALPHA = 0.20;
observer.FOC_OBS_THETA_COMP_RAD = 2.7239;
observer.stop_time = 0.05;

out_dir = fileparts(mfilename('fullpath'));

build_q_axis_current_loop_model(out_dir, common, current);
build_three_shunt_sampling_model(out_dir, common, current);
build_observer_pll_model(out_dir, common, observer);

fprintf('Models created in: %s\n', out_dir);
end

function build_q_axis_current_loop_model(out_dir, common, p)
mdl = 'koen_q_axis_current_loop_model';
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
    'FOC_TS_SEC=%.10g; FOC_IQ_KP=%.10g; FOC_IQ_KI=%.10g; ' ...
    'FOC_CURR_LOOP_V_LIMIT=%.10g; FOC_DQ_VOLT_LIMIT=%.10g; ' ...
    'FOC_VQ_CMD_SIGN=%.10g; FOC_RS_OHM=%.10g; FOC_LQ_H=%.10g; FOC_FLUX_WB=%.10g; ' ...
    'OMEGA_E_RAD_S=%.10g;'], ...
    p.FOC_TS_SEC, p.FOC_IQ_KP, p.FOC_IQ_KI, ...
    p.FOC_CURR_LOOP_V_LIMIT, p.FOC_DQ_VOLT_LIMIT, ...
    p.FOC_VQ_CMD_SIGN, p.FOC_RS_OHM, p.FOC_LQ_H, p.FOC_FLUX_WB, ...
    p.OMEGA_E_RAD_S));

add_block('simulink/Sources/In1', [mdl '/iq_ref_a'], 'Position', [35 110 65 124]);
add_block('simulink/Sources/Constant', [mdl '/omega_e_rad_s'], ...
    'Value', 'OMEGA_E_RAD_S', 'Position', [35 200 100 220]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/PI_q_discrete'], ...
    'Position', [130 85 330 165]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/sign_q'], ...
    'Position', [390 95 480 155]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/vq_limit'], ...
    'Position', [540 95 680 155]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/QAxis_RL_BackEMF_Plant'], ...
    'Position', [760 80 980 180]);

annotate_subsystem([mdl '/PI_q_discrete'], ...
    'PI_q_discrete：q轴离散 PI。用于调参，不是完整 dq 电流环。真实代码主链见 Control/foc_current_loop.c：PIController_Run -> FOC_VQ_CMD_SIGN -> foc_current_loop_limit_vector(vd,vq) -> InvPark。');
annotate_subsystem([mdl '/sign_q'], 'sign_q：把 q 轴 PI 输出乘上 FOC_VQ_CMD_SIGN。');
annotate_subsystem([mdl '/vq_limit'], 'vq_limit：对 q 轴输出做电压限幅，并输出饱和标志。');
annotate_subsystem([mdl '/QAxis_RL_BackEMF_Plant'], 'QAxis_RL_BackEMF_Plant：Rq + Lq 对象，并加入 omega_e * psi_f 反电动势扰动。');

populate_qaxis_pi_analysis_subsystem([mdl '/PI_q_discrete']);
populate_sign_axis_subsystem([mdl '/sign_q'], 'FOC_VQ_CMD_SIGN');
populate_axis_limit_with_flag_subsystem([mdl '/vq_limit']);
populate_qaxis_rl_bemf_plant_subsystem([mdl '/QAxis_RL_BackEMF_Plant']);

add_block('simulink/Sinks/Out1', [mdl '/iq_a'], 'Position', [1045 95 1075 109]);
add_block('simulink/Sinks/Out1', [mdl '/vq_cmd_v'], 'Position', [1045 130 1075 144]);
add_block('simulink/Sinks/Out1', [mdl '/iq_err_a'], 'Position', [1045 165 1075 179]);
add_block('simulink/Sinks/Out1', [mdl '/vq_sat_flag'], 'Position', [1045 200 1075 214]);

safe_add_line(mdl, 'iq_ref_a/1', 'PI_q_discrete/1');
safe_add_line(mdl, 'QAxis_RL_BackEMF_Plant/1', 'PI_q_discrete/2');
safe_add_line(mdl, 'PI_q_discrete/1', 'iq_err_a/1');
safe_add_line(mdl, 'PI_q_discrete/2', 'sign_q/1');
safe_add_line(mdl, 'sign_q/1', 'vq_limit/1');
safe_add_line(mdl, 'vq_limit/1', 'vq_cmd_v/1');
safe_add_line(mdl, 'vq_limit/2', 'vq_sat_flag/1');
safe_add_line(mdl, 'vq_limit/1', 'QAxis_RL_BackEMF_Plant/1');
safe_add_line(mdl, 'omega_e_rad_s/1', 'QAxis_RL_BackEMF_Plant/2');
safe_add_line(mdl, 'QAxis_RL_BackEMF_Plant/1', 'iq_a/1');

add_annotation(mdl, [20 30 650 48], ...
    'q轴电流环简化模型：iq_ref -> 离散 PI -> 极性映射 -> 电压限幅 -> Rq/Lq 对象 + 反电动势。');
arrange_and_save(mdl, mdl_path);
end

function build_three_shunt_sampling_model(out_dir, common, p)
mdl = 'koen_three_shunt_sampling_model';
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
    'FOC_PWM_HALF_ARR=%.10g; SAMPLE_MARGIN=120; ' ...
    'SAMPLE_LEFT_LIMIT=(FOC_PWM_HALF_ARR-SAMPLE_MARGIN); ' ...
    'SAMPLE_RIGHT_LIMIT=(FOC_PWM_HALF_ARR+SAMPLE_MARGIN);'], ...
    p.FOC_PWM_HALF_ARR));

add_block('simulink/Sources/In1', [mdl '/iu_true_a'], 'Position', [35 70 65 84]);
add_block('simulink/Sources/In1', [mdl '/iv_true_a'], 'Position', [35 110 65 124]);
add_block('simulink/Sources/In1', [mdl '/iw_true_a'], 'Position', [35 150 65 164]);
add_block('simulink/Sources/In1', [mdl '/sector'], 'Position', [35 220 65 234]);
add_block('simulink/Sources/In1', [mdl '/sample_point_ccr'], 'Position', [35 260 65 274]);

add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/SamplePoint_Window_Check'], ...
    'Position', [140 210 360 290]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/ThreeShunt_Reconstruction'], ...
    'Position', [430 80 700 250]);

annotate_subsystem([mdl '/SamplePoint_Window_Check'], ...
    'SamplePoint_Window_Check：检查采样点是否落在安全窗口内，窗口外则 fallback。');
annotate_subsystem([mdl '/ThreeShunt_Reconstruction'], ...
    'ThreeShunt_Reconstruction：严格按 Control/foc_sampling.c 的 sector 1/6、2/3、4/5 逻辑选择两相并重构第三相。');

populate_sampling_window_check_subsystem([mdl '/SamplePoint_Window_Check']);
populate_three_shunt_reconstruction_analysis_subsystem([mdl '/ThreeShunt_Reconstruction']);

add_block('simulink/Sinks/Out1', [mdl '/iu_meas_a'], 'Position', [770 90 800 104]);
add_block('simulink/Sinks/Out1', [mdl '/iv_meas_a'], 'Position', [770 125 800 139]);
add_block('simulink/Sinks/Out1', [mdl '/iw_meas_a'], 'Position', [770 160 800 174]);
add_block('simulink/Sinks/Out1', [mdl '/trusted_pair'], 'Position', [770 195 800 209]);
add_block('simulink/Sinks/Out1', [mdl '/midpoint_ok'], 'Position', [770 230 800 244]);
add_block('simulink/Sinks/Out1', [mdl '/fallback_flag'], 'Position', [770 265 800 279]);

safe_add_line(mdl, 'iu_true_a/1', 'ThreeShunt_Reconstruction/1');
safe_add_line(mdl, 'iv_true_a/1', 'ThreeShunt_Reconstruction/2');
safe_add_line(mdl, 'iw_true_a/1', 'ThreeShunt_Reconstruction/3');
safe_add_line(mdl, 'sector/1', 'ThreeShunt_Reconstruction/4');
safe_add_line(mdl, 'sample_point_ccr/1', 'SamplePoint_Window_Check/1');
safe_add_line(mdl, 'ThreeShunt_Reconstruction/1', 'iu_meas_a/1');
safe_add_line(mdl, 'ThreeShunt_Reconstruction/2', 'iv_meas_a/1');
safe_add_line(mdl, 'ThreeShunt_Reconstruction/3', 'iw_meas_a/1');
safe_add_line(mdl, 'ThreeShunt_Reconstruction/4', 'trusted_pair/1');
safe_add_line(mdl, 'SamplePoint_Window_Check/1', 'midpoint_ok/1');
safe_add_line(mdl, 'SamplePoint_Window_Check/2', 'fallback_flag/1');

add_annotation(mdl, [20 30 640 48], ...
    '三下桥采样与重构模型：真实三相电流 + sector + sample point -> 两相测量 + 第三相重构。');
arrange_and_save(mdl, mdl_path);
end

function build_observer_pll_model(out_dir, common, p)
mdl = 'koen_observer_pll_model';
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
    'FOC_TS_SEC=%.10g; FOC_RS_OHM=%.10g; FOC_LS_H=%.10g; ' ...
    'FOC_FLUX_WB=%.10g; FOC_FLUX_LIMIT_RATIO=%.10g; ' ...
    'FOC_OBS_PLL_KP=%.10g; FOC_OBS_PLL_KI=%.10g; ' ...
    'FOC_OBS_PLL_INTEGRAL_LIMIT=%.10g; FOC_OBS_SPEED_LIMIT_RAD_S=%.10g; ' ...
    'FOC_OBS_SPEED_FILTER_ALPHA=%.10g; FOC_OBS_THETA_COMP_RAD=%.10g; FOC_TWO_PI=6.283185307179586;'], ...
    p.FOC_TS_SEC, p.FOC_RS_OHM, p.FOC_LS_H, ...
    p.FOC_FLUX_WB, p.FOC_FLUX_LIMIT_RATIO, ...
    p.FOC_OBS_PLL_KP, p.FOC_OBS_PLL_KI, ...
    p.FOC_OBS_PLL_INTEGRAL_LIMIT, p.FOC_OBS_SPEED_LIMIT_RAD_S, ...
    p.FOC_OBS_SPEED_FILTER_ALPHA, p.FOC_OBS_THETA_COMP_RAD));

add_block('simulink/Sources/In1', [mdl '/Valpha'], 'Position', [40 80 70 94]);
add_block('simulink/Sources/In1', [mdl '/Vbeta'], 'Position', [40 120 70 134]);
add_block('simulink/Sources/In1', [mdl '/Ialpha'], 'Position', [40 160 70 174]);
add_block('simulink/Sources/In1', [mdl '/Ibeta'], 'Position', [40 200 70 214]);

add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Flux_Observer_CodeAligned'], ...
    'Position', [160 90 400 240]);
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/PLL_CodeAligned'], ...
    'Position', [500 90 760 240]);

annotate_subsystem([mdl '/Flux_Observer_CodeAligned'], ...
    'Flux_Observer_CodeAligned：按 FluxObserver_PLL.c 更新定子磁链、扣除 Ls*i，并加入磁链幅值限幅。');
annotate_subsystem([mdl '/PLL_CodeAligned'], ...
    'PLL_CodeAligned：按 FluxObserver_PLL.c 计算归一化相位误差、积分限幅、速度限幅、速度滤波和 theta wrap。');

populate_flux_observer_code_aligned_subsystem([mdl '/Flux_Observer_CodeAligned']);
populate_pll_code_aligned_subsystem([mdl '/PLL_CodeAligned']);

add_block('simulink/Sinks/Out1', [mdl '/theta'], 'Position', [845 90 875 104]);
add_block('simulink/Sinks/Out1', [mdl '/speed'], 'Position', [845 125 875 139]);
add_block('simulink/Sinks/Out1', [mdl '/flux_alpha'], 'Position', [845 160 875 174]);
add_block('simulink/Sinks/Out1', [mdl '/flux_beta'], 'Position', [845 195 875 209]);
add_block('simulink/Sinks/Out1', [mdl '/err'], 'Position', [845 230 875 244]);

safe_add_line(mdl, 'Valpha/1', 'Flux_Observer_CodeAligned/1');
safe_add_line(mdl, 'Vbeta/1', 'Flux_Observer_CodeAligned/2');
safe_add_line(mdl, 'Ialpha/1', 'Flux_Observer_CodeAligned/3');
safe_add_line(mdl, 'Ibeta/1', 'Flux_Observer_CodeAligned/4');
safe_add_line(mdl, 'Flux_Observer_CodeAligned/1', 'PLL_CodeAligned/1');
safe_add_line(mdl, 'Flux_Observer_CodeAligned/2', 'PLL_CodeAligned/2');
safe_add_line(mdl, 'Flux_Observer_CodeAligned/3', 'PLL_CodeAligned/3');
safe_add_line(mdl, 'PLL_CodeAligned/1', 'theta/1');
safe_add_line(mdl, 'PLL_CodeAligned/2', 'speed/1');
safe_add_line(mdl, 'Flux_Observer_CodeAligned/1', 'flux_alpha/1');
safe_add_line(mdl, 'Flux_Observer_CodeAligned/2', 'flux_beta/1');
safe_add_line(mdl, 'PLL_CodeAligned/3', 'err/1');

add_annotation(mdl, [20 30 660 48], ...
    'observer + PLL 模型：按 FluxObserver_PLL.c 关键公式实现，用于分析低速估算与锁相稳定性。');
arrange_and_save(mdl, mdl_path);
end

function populate_qaxis_pi_analysis_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/iq_ref_a'], 'Position', [25 48 55 62]);
add_block('simulink/Sources/In1', [subsys '/iq_meas_a'], 'Position', [25 98 55 112]);
add_block('simulink/Sources/Constant', [subsys '/dt_sec'], ...
    'Value', 'FOC_TS_SEC', 'Position', [25 148 75 168]);
add_block('simulink/Math Operations/Sum', [subsys '/iq_err_sum'], ...
    'Inputs', '+-', 'Position', [95 68 125 92]);
add_block('simulink/Ports & Subsystems/Subsystem', [subsys '/PI_q'], ...
    'Position', [160 55 390 145]);

annotate_subsystem([subsys '/PI_q'], 'PI_q：离散 PI，用于 q 轴跟踪误差调节。');
populate_pi_controller_run_subsystem([subsys '/PI_q'], 'FOC_IQ_KP', 'FOC_IQ_KI');

add_block('simulink/Sinks/Out1', [subsys '/iq_err_a'], 'Position', [455 70 485 84]);
add_block('simulink/Sinks/Out1', [subsys '/vq_pi_v'], 'Position', [455 110 485 124]);

safe_add_line(subsys, 'iq_ref_a/1', 'iq_err_sum/1');
safe_add_line(subsys, 'iq_meas_a/1', 'iq_err_sum/2');
safe_add_line(subsys, 'iq_ref_a/1', 'PI_q/1');
safe_add_line(subsys, 'iq_meas_a/1', 'PI_q/2');
safe_add_line(subsys, 'dt_sec/1', 'PI_q/3');
safe_add_line(subsys, 'iq_err_sum/1', 'iq_err_a/1');
safe_add_line(subsys, 'PI_q/1', 'vq_pi_v/1');

add_annotation(subsys, [20 10 350 30], ...
    'q轴离散 PI：输出误差和 PI 电压结果，便于观察跟踪误差。');
arrange_subsystem(subsys);
end

function populate_axis_limit_with_flag_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/vq_in'], 'Position', [25 68 55 82]);
add_block('simulink/Discontinuities/Saturation', [subsys '/vq_sat'], ...
    'UpperLimit', 'FOC_DQ_VOLT_LIMIT', 'LowerLimit', '-FOC_DQ_VOLT_LIMIT', 'Position', [110 58 170 92]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/upper_hit'], ...
    'Operator', '>=', 'Position', [215 38 265 68]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/lower_hit'], ...
    'Operator', '<=', 'Position', [215 98 265 128]);
add_block('simulink/Sources/Constant', [subsys '/upper_limit'], ...
    'Value', 'FOC_DQ_VOLT_LIMIT', 'Position', [215 10 280 30]);
add_block('simulink/Sources/Constant', [subsys '/lower_limit'], ...
    'Value', '-FOC_DQ_VOLT_LIMIT', 'Position', [215 150 285 170]);
add_block('simulink/Logic and Bit Operations/Logical Operator', [subsys '/sat_or'], ...
    'Operator', 'OR', 'Inputs', '2', 'Position', [305 65 355 105]);
add_block('simulink/Sinks/Out1', [subsys '/vq_out'], 'Position', [405 60 435 74]);
add_block('simulink/Sinks/Out1', [subsys '/sat_flag'], 'Position', [405 100 435 114]);

safe_add_line(subsys, 'vq_in/1', 'vq_sat/1');
safe_add_line(subsys, 'vq_sat/1', 'vq_out/1');
safe_add_line(subsys, 'vq_sat/1', 'upper_hit/1');
safe_add_line(subsys, 'upper_limit/1', 'upper_hit/2');
safe_add_line(subsys, 'vq_sat/1', 'lower_hit/1');
safe_add_line(subsys, 'lower_limit/1', 'lower_hit/2');
safe_add_line(subsys, 'upper_hit/1', 'sat_or/1');
safe_add_line(subsys, 'lower_hit/1', 'sat_or/2');
safe_add_line(subsys, 'sat_or/1', 'sat_flag/1');

add_annotation(subsys, [20 10 340 30], '单轴电压限幅：输出限幅后的 vq，并标记是否发生饱和。');
arrange_subsystem(subsys);
end

function populate_qaxis_rl_bemf_plant_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/vq_cmd_v'], 'Position', [25 48 55 62]);
add_block('simulink/Sources/In1', [subsys '/omega_e_rad_s'], 'Position', [25 98 55 112]);
add_block('simulink/Discrete/Discrete-Time Integrator', [subsys '/iq_int'], ...
    'SampleTime', 'FOC_TS_SEC', 'InitialCondition', '0', 'Position', [315 55 355 95]);
add_block('simulink/Math Operations/Gain', [subsys '/rs_iq'], ...
    'Gain', 'FOC_RS_OHM', 'Position', [95 48 155 78]);
add_block('simulink/Math Operations/Product', [subsys '/bemf_calc'], ...
    'Position', [95 108 135 138]);
add_block('simulink/Sources/Constant', [subsys '/flux_const'], ...
    'Value', 'FOC_FLUX_WB', 'Position', [95 155 155 175]);
add_block('simulink/Math Operations/Sum', [subsys '/diq_num'], ...
    'Inputs', '+--', 'Position', [185 68 215 102]);
add_block('simulink/Math Operations/Gain', [subsys '/inv_lq'], ...
    'Gain', '1/FOC_LQ_H', 'Position', [250 68 305 98]);
add_block('simulink/Sinks/Out1', [subsys '/iq_a'], 'Position', [405 68 435 82]);

safe_add_line(subsys, 'iq_int/1', 'rs_iq/1');
safe_add_line(subsys, 'omega_e_rad_s/1', 'bemf_calc/1');
safe_add_line(subsys, 'flux_const/1', 'bemf_calc/2');
safe_add_line(subsys, 'vq_cmd_v/1', 'diq_num/1');
safe_add_line(subsys, 'rs_iq/1', 'diq_num/2');
safe_add_line(subsys, 'bemf_calc/1', 'diq_num/3');
safe_add_line(subsys, 'diq_num/1', 'inv_lq/1');
safe_add_line(subsys, 'inv_lq/1', 'iq_int/1');
safe_add_line(subsys, 'iq_int/1', 'iq_a/1');

add_annotation(subsys, [20 10 340 30], 'q轴对象：Lq*diq/dt = vq - Rq*iq - omega_e*psi_f。');
arrange_subsystem(subsys);
end

function populate_sampling_window_check_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/sample_point_ccr'], 'Position', [25 68 55 82]);
add_block('simulink/Sources/Constant', [subsys '/left_limit'], ...
    'Value', 'SAMPLE_LEFT_LIMIT', 'Position', [105 30 175 50]);
add_block('simulink/Sources/Constant', [subsys '/right_limit'], ...
    'Value', 'SAMPLE_RIGHT_LIMIT', 'Position', [105 120 180 140]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/ge_left'], ...
    'Operator', '>=', 'Position', [220 38 270 68]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/le_right'], ...
    'Operator', '<=', 'Position', [220 98 270 128]);
add_block('simulink/Logic and Bit Operations/Logical Operator', [subsys '/inside_window'], ...
    'Operator', 'AND', 'Inputs', '2', 'Position', [305 65 355 105]);
add_block('simulink/Logic and Bit Operations/Logical Operator', [subsys '/fallback_not'], ...
    'Operator', 'NOT', 'Position', [390 72 440 102]);
add_block('simulink/Sinks/Out1', [subsys '/midpoint_ok'], 'Position', [480 60 510 74]);
add_block('simulink/Sinks/Out1', [subsys '/fallback_flag'], 'Position', [480 100 510 114]);

safe_add_line(subsys, 'sample_point_ccr/1', 'ge_left/1');
safe_add_line(subsys, 'left_limit/1', 'ge_left/2');
safe_add_line(subsys, 'sample_point_ccr/1', 'le_right/1');
safe_add_line(subsys, 'right_limit/1', 'le_right/2');
safe_add_line(subsys, 'ge_left/1', 'inside_window/1');
safe_add_line(subsys, 'le_right/1', 'inside_window/2');
safe_add_line(subsys, 'inside_window/1', 'midpoint_ok/1');
safe_add_line(subsys, 'inside_window/1', 'fallback_not/1');
safe_add_line(subsys, 'fallback_not/1', 'fallback_flag/1');

add_annotation(subsys, [20 10 380 30], '采样点检查：在安全窗口内则 midpoint_ok=1，否则 fallback_flag=1。');
arrange_subsystem(subsys);
end

function populate_three_shunt_reconstruction_analysis_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/iu_true_a'], 'Position', [25 48 55 62]);
add_block('simulink/Sources/In1', [subsys '/iv_true_a'], 'Position', [25 98 55 112]);
add_block('simulink/Sources/In1', [subsys '/iw_true_a'], 'Position', [25 148 55 162]);
add_block('simulink/Sources/In1', [subsys '/sector'], 'Position', [25 208 55 222]);

add_block('simulink/Logic and Bit Operations/Compare To Constant', [subsys '/is_s1'], ...
    'const', '1', 'Position', [95 168 155 198]);
add_block('simulink/Logic and Bit Operations/Compare To Constant', [subsys '/is_s2'], ...
    'const', '2', 'Position', [95 208 155 238]);
add_block('simulink/Logic and Bit Operations/Compare To Constant', [subsys '/is_s3'], ...
    'const', '3', 'Position', [95 248 155 278]);
add_block('simulink/Logic and Bit Operations/Compare To Constant', [subsys '/is_s4'], ...
    'const', '4', 'Position', [95 288 155 318]);
add_block('simulink/Logic and Bit Operations/Compare To Constant', [subsys '/is_s5'], ...
    'const', '5', 'Position', [95 328 155 358]);
add_block('simulink/Logic and Bit Operations/Compare To Constant', [subsys '/is_s6'], ...
    'const', '6', 'Position', [95 368 155 398]);
add_block('simulink/Logic and Bit Operations/Logical Operator', [subsys '/is_16'], ...
    'Operator', 'OR', 'Inputs', '2', 'Position', [190 190 240 236]);
add_block('simulink/Logic and Bit Operations/Logical Operator', [subsys '/is_23'], ...
    'Operator', 'OR', 'Inputs', '2', 'Position', [190 255 240 301]);
add_block('simulink/Logic and Bit Operations/Logical Operator', [subsys '/is_45'], ...
    'Operator', 'OR', 'Inputs', '2', 'Position', [190 320 240 366]);

add_block('simulink/Math Operations/Sum', [subsys '/sum_vw'], ...
    'Inputs', '++', 'Position', [285 78 315 108]);
add_block('simulink/Math Operations/Gain', [subsys '/recon_u'], ...
    'Gain', '-1', 'Position', [345 78 395 108]);
add_block('simulink/Math Operations/Sum', [subsys '/sum_uw'], ...
    'Inputs', '++', 'Position', [285 128 315 158]);
add_block('simulink/Math Operations/Gain', [subsys '/recon_v'], ...
    'Gain', '-1', 'Position', [345 128 395 158]);
add_block('simulink/Math Operations/Sum', [subsys '/sum_uv'], ...
    'Inputs', '++', 'Position', [285 178 315 208]);
add_block('simulink/Math Operations/Gain', [subsys '/recon_w'], ...
    'Gain', '-1', 'Position', [345 178 395 208]);

add_block('simulink/Signal Routing/Switch', [subsys '/iu_switch'], ...
    'Threshold', '0.5', 'Criteria', 'u2 ~= 0', 'Position', [455 60 505 130]);
add_block('simulink/Signal Routing/Switch', [subsys '/iv_switch'], ...
    'Threshold', '0.5', 'Criteria', 'u2 ~= 0', 'Position', [455 120 505 190]);
add_block('simulink/Signal Routing/Switch', [subsys '/iw_switch'], ...
    'Threshold', '0.5', 'Criteria', 'u2 ~= 0', 'Position', [455 180 505 250]);

add_block('simulink/Sources/Constant', [subsys '/pair_vw'], 'Value', '1', 'Position', [455 285 485 305]);
add_block('simulink/Sources/Constant', [subsys '/pair_uw'], 'Value', '2', 'Position', [495 285 525 305]);
add_block('simulink/Sources/Constant', [subsys '/pair_uv'], 'Value', '3', 'Position', [535 285 565 305]);
add_block('simulink/Signal Routing/Switch', [subsys '/pair_switch_1'], ...
    'Threshold', '0.5', 'Criteria', 'u2 ~= 0', 'Position', [595 270 645 320]);
add_block('simulink/Signal Routing/Switch', [subsys '/pair_switch_2'], ...
    'Threshold', '0.5', 'Criteria', 'u2 ~= 0', 'Position', [675 270 725 320]);

add_block('simulink/Sinks/Out1', [subsys '/iu_meas_a'], 'Position', [780 80 810 94]);
add_block('simulink/Sinks/Out1', [subsys '/iv_meas_a'], 'Position', [780 120 810 134]);
add_block('simulink/Sinks/Out1', [subsys '/iw_meas_a'], 'Position', [780 160 810 174]);
add_block('simulink/Sinks/Out1', [subsys '/trusted_pair'], 'Position', [780 290 810 304]);

safe_add_line(subsys, 'sector/1', 'is_s1/1');
safe_add_line(subsys, 'sector/1', 'is_s2/1');
safe_add_line(subsys, 'sector/1', 'is_s3/1');
safe_add_line(subsys, 'sector/1', 'is_s4/1');
safe_add_line(subsys, 'sector/1', 'is_s5/1');
safe_add_line(subsys, 'sector/1', 'is_s6/1');
safe_add_line(subsys, 'is_s1/1', 'is_16/1');
safe_add_line(subsys, 'is_s6/1', 'is_16/2');
safe_add_line(subsys, 'is_s2/1', 'is_23/1');
safe_add_line(subsys, 'is_s3/1', 'is_23/2');
safe_add_line(subsys, 'is_s4/1', 'is_45/1');
safe_add_line(subsys, 'is_s5/1', 'is_45/2');

safe_add_line(subsys, 'iv_true_a/1', 'sum_vw/1');
safe_add_line(subsys, 'iw_true_a/1', 'sum_vw/2');
safe_add_line(subsys, 'sum_vw/1', 'recon_u/1');
safe_add_line(subsys, 'iu_true_a/1', 'sum_uw/1');
safe_add_line(subsys, 'iw_true_a/1', 'sum_uw/2');
safe_add_line(subsys, 'sum_uw/1', 'recon_v/1');
safe_add_line(subsys, 'iu_true_a/1', 'sum_uv/1');
safe_add_line(subsys, 'iv_true_a/1', 'sum_uv/2');
safe_add_line(subsys, 'sum_uv/1', 'recon_w/1');

safe_add_line(subsys, 'recon_u/1', 'iu_switch/1');
safe_add_line(subsys, 'is_16/1', 'iu_switch/2');
safe_add_line(subsys, 'iu_true_a/1', 'iu_switch/3');
safe_add_line(subsys, 'recon_v/1', 'iv_switch/1');
safe_add_line(subsys, 'is_23/1', 'iv_switch/2');
safe_add_line(subsys, 'iv_true_a/1', 'iv_switch/3');
safe_add_line(subsys, 'recon_w/1', 'iw_switch/1');
safe_add_line(subsys, 'is_45/1', 'iw_switch/2');
safe_add_line(subsys, 'iw_true_a/1', 'iw_switch/3');

safe_add_line(subsys, 'iu_switch/1', 'iu_meas_a/1');
safe_add_line(subsys, 'iv_switch/1', 'iv_meas_a/1');
safe_add_line(subsys, 'iw_switch/1', 'iw_meas_a/1');

safe_add_line(subsys, 'pair_vw/1', 'pair_switch_1/1');
safe_add_line(subsys, 'is_16/1', 'pair_switch_1/2');
safe_add_line(subsys, 'pair_uw/1', 'pair_switch_1/3');
safe_add_line(subsys, 'pair_switch_1/1', 'pair_switch_2/1');
safe_add_line(subsys, 'is_23/1', 'pair_switch_2/2');
safe_add_line(subsys, 'pair_uv/1', 'pair_switch_2/3');
safe_add_line(subsys, 'pair_switch_2/1', 'trusted_pair/1');

add_annotation(subsys, [20 10 520 30], ...
    '三下桥重构：sector 1/6 选 VW，2/3 选 UW，4/5/default 选 UV，再由和为零重构第三相。');
arrange_subsystem(subsys);
end

function populate_flux_observer_code_aligned_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/Valpha'], 'Position', [25 40 55 54]);
add_block('simulink/Sources/In1', [subsys '/Vbeta'], 'Position', [25 90 55 104]);
add_block('simulink/Sources/In1', [subsys '/Ialpha'], 'Position', [25 140 55 154]);
add_block('simulink/Sources/In1', [subsys '/Ibeta'], 'Position', [25 190 55 204]);

add_block('simulink/Discrete/Unit Delay', [subsys '/s_flux_s_alpha_prev'], ...
    'SampleTime', 'FOC_TS_SEC', 'InitialCondition', 'FOC_FLUX_WB', 'Position', [410 45 450 75]);
add_block('simulink/Discrete/Unit Delay', [subsys '/s_flux_s_beta_prev'], ...
    'SampleTime', 'FOC_TS_SEC', 'InitialCondition', '0', 'Position', [410 95 450 125]);

add_block('simulink/Math Operations/Gain', [subsys '/Rs_Ialpha'], ...
    'Gain', 'FOC_RS_OHM', 'Position', [95 130 155 150]);
add_block('simulink/Math Operations/Gain', [subsys '/Rs_Ibeta'], ...
    'Gain', 'FOC_RS_OHM', 'Position', [95 180 155 200]);
add_block('simulink/Math Operations/Sum', [subsys '/psi_alpha_dot'], ...
    'Inputs', '+-', 'Position', [190 48 220 82]);
add_block('simulink/Math Operations/Sum', [subsys '/psi_beta_dot'], ...
    'Inputs', '+-', 'Position', [190 98 220 132]);
add_block('simulink/Math Operations/Gain', [subsys '/Ts_gain_alpha'], ...
    'Gain', 'FOC_TS_SEC', 'Position', [250 48 320 82]);
add_block('simulink/Math Operations/Gain', [subsys '/Ts_gain_beta'], ...
    'Gain', 'FOC_TS_SEC', 'Position', [250 98 320 132]);
add_block('simulink/Math Operations/Sum', [subsys '/s_flux_alpha_cand'], ...
    'Inputs', '++', 'Position', [350 45 380 75]);
add_block('simulink/Math Operations/Sum', [subsys '/s_flux_beta_cand'], ...
    'Inputs', '++', 'Position', [350 95 380 125]);

add_block('simulink/Math Operations/Gain', [subsys '/Ls_Ialpha'], ...
    'Gain', 'FOC_LS_H', 'Position', [350 150 410 170]);
add_block('simulink/Math Operations/Gain', [subsys '/Ls_Ibeta'], ...
    'Gain', 'FOC_LS_H', 'Position', [350 190 410 210]);
add_block('simulink/Math Operations/Sum', [subsys '/flux_alpha_raw'], ...
    'Inputs', '+-', 'Position', [470 48 500 82]);
add_block('simulink/Math Operations/Sum', [subsys '/flux_beta_raw'], ...
    'Inputs', '+-', 'Position', [470 98 500 132]);

add_block('simulink/Math Operations/Product', [subsys '/flux_alpha_sq'], ...
    'Position', [535 38 575 72]);
add_block('simulink/Math Operations/Product', [subsys '/flux_beta_sq'], ...
    'Position', [535 88 575 122]);
add_block('simulink/Math Operations/Sum', [subsys '/flux_mag_sq'], ...
    'Inputs', '++', 'Position', [605 65 635 95]);
add_block('simulink/Math Operations/Math Function', [subsys '/flux_mag'], ...
    'Operator', 'sqrt', 'Position', [665 62 705 98]);
add_block('simulink/Sources/Constant', [subsys '/flux_limit'], ...
    'Value', '(FOC_FLUX_WB*FOC_FLUX_LIMIT_RATIO)', 'Position', [665 120 760 140]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/need_limit'], ...
    'Operator', '>', 'Position', [795 70 845 100]);
add_block('simulink/Sources/Constant', [subsys '/eps'], ...
    'Value', '1e-6', 'Position', [795 120 835 140]);
add_block('simulink/Math Operations/MinMax', [subsys '/flux_mag_safe'], ...
    'Function', 'max', 'Inputs', '2', 'Position', [870 110 920 150]);
add_block('simulink/Math Operations/Divide', [subsys '/scale_raw'], ...
    'Position', [955 62 995 96]);
add_block('simulink/Sources/Constant', [subsys '/one'], ...
    'Value', '1', 'Position', [955 120 985 140]);
add_block('simulink/Signal Routing/Switch', [subsys '/scale_switch'], ...
    'Threshold', '0.5', 'Criteria', 'u2 ~= 0', 'Position', [1025 55 1075 129]);
add_block('simulink/Math Operations/Product', [subsys '/flux_alpha_limited'], ...
    'Position', [1110 42 1150 76]);
add_block('simulink/Math Operations/Product', [subsys '/flux_beta_limited'], ...
    'Position', [1110 92 1150 126]);
add_block('simulink/Math Operations/Sum', [subsys '/s_flux_alpha_next'], ...
    'Inputs', '++', 'Position', [1190 145 1220 175]);
add_block('simulink/Math Operations/Sum', [subsys '/s_flux_beta_next'], ...
    'Inputs', '++', 'Position', [1190 195 1220 225]);

add_block('simulink/Sinks/Out1', [subsys '/flux_alpha'], 'Position', [1270 50 1300 64]);
add_block('simulink/Sinks/Out1', [subsys '/flux_beta'], 'Position', [1270 100 1300 114]);
add_block('simulink/Sinks/Out1', [subsys '/flux_mag_out'], 'Position', [1270 150 1300 164]);

safe_add_line(subsys, 'Ialpha/1', 'Rs_Ialpha/1');
safe_add_line(subsys, 'Ibeta/1', 'Rs_Ibeta/1');
safe_add_line(subsys, 'Valpha/1', 'psi_alpha_dot/1');
safe_add_line(subsys, 'Rs_Ialpha/1', 'psi_alpha_dot/2');
safe_add_line(subsys, 'Vbeta/1', 'psi_beta_dot/1');
safe_add_line(subsys, 'Rs_Ibeta/1', 'psi_beta_dot/2');
safe_add_line(subsys, 'psi_alpha_dot/1', 'Ts_gain_alpha/1');
safe_add_line(subsys, 'psi_beta_dot/1', 'Ts_gain_beta/1');
safe_add_line(subsys, 's_flux_s_alpha_prev/1', 's_flux_alpha_cand/1');
safe_add_line(subsys, 'Ts_gain_alpha/1', 's_flux_alpha_cand/2');
safe_add_line(subsys, 's_flux_s_beta_prev/1', 's_flux_beta_cand/1');
safe_add_line(subsys, 'Ts_gain_beta/1', 's_flux_beta_cand/2');
safe_add_line(subsys, 'Ialpha/1', 'Ls_Ialpha/1');
safe_add_line(subsys, 'Ibeta/1', 'Ls_Ibeta/1');
safe_add_line(subsys, 's_flux_alpha_cand/1', 'flux_alpha_raw/1');
safe_add_line(subsys, 'Ls_Ialpha/1', 'flux_alpha_raw/2');
safe_add_line(subsys, 's_flux_beta_cand/1', 'flux_beta_raw/1');
safe_add_line(subsys, 'Ls_Ibeta/1', 'flux_beta_raw/2');
safe_add_line(subsys, 'flux_alpha_raw/1', 'flux_alpha_sq/1');
safe_add_line(subsys, 'flux_alpha_raw/1', 'flux_alpha_sq/2');
safe_add_line(subsys, 'flux_beta_raw/1', 'flux_beta_sq/1');
safe_add_line(subsys, 'flux_beta_raw/1', 'flux_beta_sq/2');
safe_add_line(subsys, 'flux_alpha_sq/1', 'flux_mag_sq/1');
safe_add_line(subsys, 'flux_beta_sq/1', 'flux_mag_sq/2');
safe_add_line(subsys, 'flux_mag_sq/1', 'flux_mag/1');
safe_add_line(subsys, 'flux_mag/1', 'need_limit/1');
safe_add_line(subsys, 'flux_limit/1', 'need_limit/2');
safe_add_line(subsys, 'flux_mag/1', 'flux_mag_safe/1');
safe_add_line(subsys, 'eps/1', 'flux_mag_safe/2');
safe_add_line(subsys, 'flux_limit/1', 'scale_raw/1');
safe_add_line(subsys, 'flux_mag_safe/1', 'scale_raw/2');
safe_add_line(subsys, 'scale_raw/1', 'scale_switch/1');
safe_add_line(subsys, 'need_limit/1', 'scale_switch/2');
safe_add_line(subsys, 'one/1', 'scale_switch/3');
safe_add_line(subsys, 'flux_alpha_raw/1', 'flux_alpha_limited/1');
safe_add_line(subsys, 'flux_beta_raw/1', 'flux_beta_limited/1');
safe_add_line(subsys, 'scale_switch/1', 'flux_alpha_limited/2');
safe_add_line(subsys, 'scale_switch/1', 'flux_beta_limited/2');
safe_add_line(subsys, 'flux_alpha_limited/1', 's_flux_alpha_next/1');
safe_add_line(subsys, 'Ls_Ialpha/1', 's_flux_alpha_next/2');
safe_add_line(subsys, 'flux_beta_limited/1', 's_flux_beta_next/1');
safe_add_line(subsys, 'Ls_Ibeta/1', 's_flux_beta_next/2');
safe_add_line(subsys, 's_flux_alpha_next/1', 's_flux_s_alpha_prev/1');
safe_add_line(subsys, 's_flux_beta_next/1', 's_flux_s_beta_prev/1');
safe_add_line(subsys, 'flux_alpha_limited/1', 'flux_alpha/1');
safe_add_line(subsys, 'flux_beta_limited/1', 'flux_beta/1');
safe_add_line(subsys, 'flux_mag/1', 'flux_mag_out/1');

add_annotation(subsys, [20 10 620 30], ...
    '磁链观测器：积分 u-R*i，扣除 Ls*i 得到转子磁链，再按上限做幅值限制。');
arrange_subsystem(subsys);
end

function populate_pll_code_aligned_subsystem(subsys)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/flux_alpha'], 'Position', [25 40 55 54]);
add_block('simulink/Sources/In1', [subsys '/flux_beta'], 'Position', [25 90 55 104]);
add_block('simulink/Sources/In1', [subsys '/flux_mag'], 'Position', [25 140 55 154]);

add_block('simulink/Discrete/Unit Delay', [subsys '/theta_prev'], ...
    'SampleTime', 'FOC_TS_SEC', 'InitialCondition', '0', 'Position', [470 30 510 60]);
add_block('simulink/Discrete/Unit Delay', [subsys '/pll_int_prev'], ...
    'SampleTime', 'FOC_TS_SEC', 'InitialCondition', '0', 'Position', [470 120 510 150]);
add_block('simulink/Discrete/Unit Delay', [subsys '/speed_filt_prev'], ...
    'SampleTime', 'FOC_TS_SEC', 'InitialCondition', '0', 'Position', [470 210 510 240]);

add_block('simulink/Math Operations/Trigonometric Function', [subsys '/sin_theta'], ...
    'Operator', 'sin', 'Position', [95 20 135 50]);
add_block('simulink/Math Operations/Trigonometric Function', [subsys '/cos_theta'], ...
    'Operator', 'cos', 'Position', [95 60 135 90]);
add_block('simulink/Math Operations/Product', [subsys '/flux_beta_cos'], ...
    'Position', [175 58 215 88]);
add_block('simulink/Math Operations/Product', [subsys '/flux_alpha_sin'], ...
    'Position', [175 18 215 48]);
add_block('simulink/Math Operations/Sum', [subsys '/err_raw'], ...
    'Inputs', '+-', 'Position', [250 38 280 72]);
add_block('simulink/Sources/Constant', [subsys '/eps'], ...
    'Value', '1e-6', 'Position', [250 100 290 120]);
add_block('simulink/Math Operations/MinMax', [subsys '/flux_mag_safe'], ...
    'Function', 'max', 'Inputs', '2', 'Position', [325 100 375 140]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/need_norm'], ...
    'Operator', '>', 'Position', [325 38 375 68]);
add_block('simulink/Math Operations/Divide', [subsys '/err_norm'], ...
    'Position', [405 38 445 72]);
add_block('simulink/Signal Routing/Switch', [subsys '/err_switch'], ...
    'Threshold', '0.5', 'Criteria', 'u2 ~= 0', 'Position', [475 60 525 134]);

add_block('simulink/Math Operations/Gain', [subsys '/ki_gain'], ...
    'Gain', 'FOC_OBS_PLL_KI', 'Position', [560 110 650 140]);
add_block('simulink/Math Operations/Gain', [subsys '/Ts_gain'], ...
    'Gain', 'FOC_TS_SEC', 'Position', [685 110 755 140]);
add_block('simulink/Math Operations/Sum', [subsys '/pll_int_next_raw'], ...
    'Inputs', '++', 'Position', [790 108 820 138]);
add_block('simulink/Discontinuities/Saturation', [subsys '/pll_int_clamp'], ...
    'UpperLimit', 'FOC_OBS_PLL_INTEGRAL_LIMIT', ...
    'LowerLimit', '-FOC_OBS_PLL_INTEGRAL_LIMIT', ...
    'Position', [855 100 915 140]);

add_block('simulink/Math Operations/Gain', [subsys '/kp_gain'], ...
    'Gain', 'FOC_OBS_PLL_KP', 'Position', [560 30 650 60]);
add_block('simulink/Math Operations/Sum', [subsys '/speed_raw_sum'], ...
    'Inputs', '++', 'Position', [955 48 985 78]);
add_block('simulink/Discontinuities/Saturation', [subsys '/speed_clamp'], ...
    'UpperLimit', 'FOC_OBS_SPEED_LIMIT_RAD_S', ...
    'LowerLimit', '-FOC_OBS_SPEED_LIMIT_RAD_S', ...
    'Position', [1020 40 1080 80]);

add_block('simulink/Math Operations/Sum', [subsys '/speed_filt_err'], ...
    'Inputs', '+-', 'Position', [1120 148 1150 178]);
add_block('simulink/Math Operations/Gain', [subsys '/speed_alpha'], ...
    'Gain', 'FOC_OBS_SPEED_FILTER_ALPHA', 'Position', [1185 145 1270 175]);
add_block('simulink/Math Operations/Sum', [subsys '/speed_filt_next'], ...
    'Inputs', '++', 'Position', [1305 148 1335 178]);

add_block('simulink/Math Operations/Gain', [subsys '/theta_step'], ...
    'Gain', 'FOC_TS_SEC', 'Position', [1120 40 1190 70]);
add_block('simulink/Math Operations/Sum', [subsys '/theta_sum'], ...
    'Inputs', '++', 'Position', [1230 38 1260 68]);
add_block('simulink/User-Defined Functions/Fcn', [subsys '/wrap_0_2pi'], ...
    'Expr', 'mod(u, FOC_TWO_PI)', 'Position', [1300 30 1385 70]);

add_block('simulink/Sinks/Out1', [subsys '/theta'], 'Position', [1440 40 1470 54]);
add_block('simulink/Sinks/Out1', [subsys '/speed_filt'], 'Position', [1440 155 1470 169]);
add_block('simulink/Sinks/Out1', [subsys '/err'], 'Position', [1440 95 1470 109]);

safe_add_line(subsys, 'theta_prev/1', 'sin_theta/1');
safe_add_line(subsys, 'theta_prev/1', 'cos_theta/1');
safe_add_line(subsys, 'flux_beta/1', 'flux_beta_cos/1');
safe_add_line(subsys, 'cos_theta/1', 'flux_beta_cos/2');
safe_add_line(subsys, 'flux_alpha/1', 'flux_alpha_sin/1');
safe_add_line(subsys, 'sin_theta/1', 'flux_alpha_sin/2');
safe_add_line(subsys, 'flux_beta_cos/1', 'err_raw/1');
safe_add_line(subsys, 'flux_alpha_sin/1', 'err_raw/2');
safe_add_line(subsys, 'flux_mag/1', 'need_norm/1');
safe_add_line(subsys, 'eps/1', 'need_norm/2');
safe_add_line(subsys, 'flux_mag/1', 'flux_mag_safe/1');
safe_add_line(subsys, 'eps/1', 'flux_mag_safe/2');
safe_add_line(subsys, 'err_raw/1', 'err_norm/1');
safe_add_line(subsys, 'flux_mag_safe/1', 'err_norm/2');
safe_add_line(subsys, 'err_norm/1', 'err_switch/1');
safe_add_line(subsys, 'need_norm/1', 'err_switch/2');
safe_add_line(subsys, 'err_raw/1', 'err_switch/3');

safe_add_line(subsys, 'err_switch/1', 'ki_gain/1');
safe_add_line(subsys, 'ki_gain/1', 'Ts_gain/1');
safe_add_line(subsys, 'pll_int_prev/1', 'pll_int_next_raw/1');
safe_add_line(subsys, 'Ts_gain/1', 'pll_int_next_raw/2');
safe_add_line(subsys, 'pll_int_next_raw/1', 'pll_int_clamp/1');
safe_add_line(subsys, 'pll_int_clamp/1', 'pll_int_prev/1');

safe_add_line(subsys, 'err_switch/1', 'kp_gain/1');
safe_add_line(subsys, 'pll_int_clamp/1', 'speed_raw_sum/1');
safe_add_line(subsys, 'kp_gain/1', 'speed_raw_sum/2');
safe_add_line(subsys, 'speed_raw_sum/1', 'speed_clamp/1');

safe_add_line(subsys, 'speed_clamp/1', 'speed_filt_err/1');
safe_add_line(subsys, 'speed_filt_prev/1', 'speed_filt_err/2');
safe_add_line(subsys, 'speed_filt_err/1', 'speed_alpha/1');
safe_add_line(subsys, 'speed_filt_prev/1', 'speed_filt_next/1');
safe_add_line(subsys, 'speed_alpha/1', 'speed_filt_next/2');
safe_add_line(subsys, 'speed_filt_next/1', 'speed_filt_prev/1');

safe_add_line(subsys, 'speed_clamp/1', 'theta_step/1');
safe_add_line(subsys, 'theta_prev/1', 'theta_sum/1');
safe_add_line(subsys, 'theta_step/1', 'theta_sum/2');
safe_add_line(subsys, 'theta_sum/1', 'wrap_0_2pi/1');
safe_add_line(subsys, 'wrap_0_2pi/1', 'theta_prev/1');

safe_add_line(subsys, 'wrap_0_2pi/1', 'theta/1');
safe_add_line(subsys, 'speed_filt_next/1', 'speed_filt/1');
safe_add_line(subsys, 'err_switch/1', 'err/1');

add_annotation(subsys, [20 10 620 30], ...
    'PLL：err 正交误差归一化 -> 积分限幅 -> 速度限幅 -> 一阶滤波 -> theta wrap。');
arrange_subsystem(subsys);
end

function populate_pi_controller_run_subsystem(subsys, kp_name, ki_name)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/ref'], 'Position', [25 48 55 62]);
add_block('simulink/Sources/In1', [subsys '/meas'], 'Position', [25 98 55 112]);
add_block('simulink/Sources/In1', [subsys '/dt_sec'], 'Position', [25 148 55 162]);
add_block('simulink/Discrete/Unit Delay', [subsys '/integral_prev'], ...
    'SampleTime', 'FOC_TS_SEC', 'InitialCondition', '0', 'Position', [585 210 625 240]);

add_block('simulink/Math Operations/Sum', [subsys '/err'], ...
    'Inputs', '+-', 'Position', [95 68 125 92]);
add_block('simulink/Math Operations/Gain', [subsys '/kp'], ...
    'Gain', kp_name, 'Position', [160 45 230 70]);
add_block('simulink/Math Operations/Gain', [subsys '/ki'], ...
    'Gain', ki_name, 'Position', [160 110 230 135]);
add_block('simulink/Math Operations/Product', [subsys '/ki_err_dt'], ...
    'Inputs', '2', 'Position', [270 118 310 152]);
add_block('simulink/Math Operations/Sum', [subsys '/integral_candidate'], ...
    'Inputs', '++', 'Position', [350 205 380 235]);
add_block('simulink/Discontinuities/Saturation', [subsys '/integral_clamp'], ...
    'UpperLimit', 'FOC_CURR_LOOP_V_LIMIT', ...
    'LowerLimit', '-FOC_CURR_LOOP_V_LIMIT', ...
    'Position', [420 200 475 240]);
add_block('simulink/Math Operations/Sum', [subsys '/unsat'], ...
    'Inputs', '++', 'Position', [420 78 450 102]);
add_block('simulink/Discontinuities/Saturation', [subsys '/out_clamp'], ...
    'UpperLimit', 'FOC_CURR_LOOP_V_LIMIT', ...
    'LowerLimit', '-FOC_CURR_LOOP_V_LIMIT', ...
    'Position', [495 72 550 108]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/unsat_eq_out'], ...
    'Operator', '==', 'Position', [495 125 555 155]);
add_block('simulink/Sources/Constant', [subsys '/out_max'], ...
    'Value', 'FOC_CURR_LOOP_V_LIMIT', 'Position', [495 175 555 195]);
add_block('simulink/Sources/Constant', [subsys '/out_min'], ...
    'Value', '-FOC_CURR_LOOP_V_LIMIT', 'Position', [495 255 555 275]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/unsat_gt_max'], ...
    'Operator', '>', 'Position', [585 170 645 200]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/unsat_lt_min'], ...
    'Operator', '<', 'Position', [585 250 645 280]);
add_block('simulink/Sources/Constant', [subsys '/zero'], ...
    'Value', '0', 'Position', [585 105 615 125]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/err_lt_zero'], ...
    'Operator', '<', 'Position', [675 120 735 150]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/err_gt_zero'], ...
    'Operator', '>', 'Position', [675 285 735 315]);
add_block('simulink/Logic and Bit Operations/Logical Operator', [subsys '/upper_release'], ...
    'Operator', 'AND', 'Inputs', '2', 'Position', [775 175 815 205]);
add_block('simulink/Logic and Bit Operations/Logical Operator', [subsys '/lower_release'], ...
    'Operator', 'AND', 'Inputs', '2', 'Position', [775 255 815 285]);
add_block('simulink/Logic and Bit Operations/Logical Operator', [subsys '/update_enable'], ...
    'Operator', 'OR', 'Inputs', '3', 'Position', [855 205 895 255]);
add_block('simulink/Signal Routing/Switch', [subsys '/integral_update_select'], ...
    'Threshold', '0.5', 'Criteria', 'u2 ~= 0', 'Position', [935 205 980 275]);
add_block('simulink/Sinks/Out1', [subsys '/out'], 'Position', [1035 80 1065 94]);

safe_add_line(subsys, 'ref/1', 'err/1');
safe_add_line(subsys, 'meas/1', 'err/2');
safe_add_line(subsys, 'err/1', 'kp/1');
safe_add_line(subsys, 'err/1', 'ki/1');
safe_add_line(subsys, 'ki/1', 'ki_err_dt/1');
safe_add_line(subsys, 'dt_sec/1', 'ki_err_dt/2');
safe_add_line(subsys, 'integral_prev/1', 'integral_candidate/1');
safe_add_line(subsys, 'ki_err_dt/1', 'integral_candidate/2');
safe_add_line(subsys, 'integral_candidate/1', 'integral_clamp/1');
safe_add_line(subsys, 'kp/1', 'unsat/1');
safe_add_line(subsys, 'integral_clamp/1', 'unsat/2');
safe_add_line(subsys, 'unsat/1', 'out_clamp/1');
safe_add_line(subsys, 'out_clamp/1', 'out/1');
safe_add_line(subsys, 'unsat/1', 'unsat_eq_out/1');
safe_add_line(subsys, 'out_clamp/1', 'unsat_eq_out/2');
safe_add_line(subsys, 'unsat/1', 'unsat_gt_max/1');
safe_add_line(subsys, 'out_max/1', 'unsat_gt_max/2');
safe_add_line(subsys, 'unsat/1', 'unsat_lt_min/1');
safe_add_line(subsys, 'out_min/1', 'unsat_lt_min/2');
safe_add_line(subsys, 'err/1', 'err_lt_zero/1');
safe_add_line(subsys, 'zero/1', 'err_lt_zero/2');
safe_add_line(subsys, 'err/1', 'err_gt_zero/1');
safe_add_line(subsys, 'zero/1', 'err_gt_zero/2');
safe_add_line(subsys, 'unsat_gt_max/1', 'upper_release/1');
safe_add_line(subsys, 'err_lt_zero/1', 'upper_release/2');
safe_add_line(subsys, 'unsat_lt_min/1', 'lower_release/1');
safe_add_line(subsys, 'err_gt_zero/1', 'lower_release/2');
safe_add_line(subsys, 'unsat_eq_out/1', 'update_enable/1');
safe_add_line(subsys, 'upper_release/1', 'update_enable/2');
safe_add_line(subsys, 'lower_release/1', 'update_enable/3');
safe_add_line(subsys, 'integral_clamp/1', 'integral_update_select/1');
safe_add_line(subsys, 'update_enable/1', 'integral_update_select/2');
safe_add_line(subsys, 'integral_prev/1', 'integral_update_select/3');
safe_add_line(subsys, 'integral_update_select/1', 'integral_prev/1');

add_annotation(subsys, [20 10 420 30], ...
    '严格镜像 PIController_Run()：积分限幅、输出限幅、以及按 unsat/out/err 条件更新积分。');
arrange_subsystem(subsys);
end

function populate_sign_axis_subsystem(subsys, gain_name)
clear_subsystem(subsys);

add_block('simulink/Sources/In1', [subsys '/axis_in'], 'Position', [25 48 55 62]);
add_block('simulink/Math Operations/Gain', [subsys '/sign_gain'], ...
    'Gain', gain_name, 'Position', [100 40 170 70]);
add_block('simulink/Sinks/Out1', [subsys '/axis_out'], 'Position', [225 48 255 62]);

safe_add_line(subsys, 'axis_in/1', 'sign_gain/1');
safe_add_line(subsys, 'sign_gain/1', 'axis_out/1');

add_annotation(subsys, [20 10 240 30], '单轴极性映射。');
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
