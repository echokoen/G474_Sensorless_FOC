function results = run_current_loop_tests()
%RUN_CURRENT_LOOP_TESTS Run basic PMSM current-loop discrete model tests.

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(script_dir));
model_name = 'pmsm_current_loop_discrete';
model_path = fullfile(repo_root, 'model', 'simulink', [model_name '.slx']);
param_path = fullfile(script_dir, 'pmsm_current_loop_params.m');
result_path = fullfile(repo_root, 'model', 'simulink', 'current_loop_test_results.mat');

if ~exist(model_path, 'file')
    error('Model not found: %s', model_path);
end

evalin('base', sprintf('run(''%s'');', strrep(param_path, '''', '''''')));
load_system(model_path);

tests = local_test_table();
results = struct([]);

for k = 1:numel(tests)
    t = tests(k);
    evalin('base', sprintf('run(''%s'');', strrep(param_path, '''', '''''')));
    assignin('base', 'pmsm_current_loop_keep_overrides', 1);
    assignin('base', 'id_ref_before', t.id_before);
    assignin('base', 'id_ref_after', t.id_after);
    assignin('base', 'iq_ref_before', t.iq_before);
    assignin('base', 'iq_ref_after', t.iq_after);
    assignin('base', 'sim_mech_speed_rad_s', t.mech_speed_rad_s);
    assignin('base', 'use_svpwm_equivalent', t.use_svpwm);
    assignin('base', 'Vbus', t.vbus);

    out = sim(model_name, ...
        'StopTime', num2str(t.stop_time), ...
        'ReturnWorkspaceOutputs', 'on');

    log = out.get('current_loop_log');
    summary = local_summarize(log);

    results(k).name = t.name;
    results(k).description = t.description;
    results(k).settings = t;
    results(k).summary = summary;
    results(k).log = log;

    fprintf('%s: id_final=%.3f A, iq_final=%.3f A, max|vdq|=%.3f V, limit_seen=%u\n', ...
        t.name, summary.id_final, summary.iq_final, summary.max_vdq, summary.limit_seen);
end

assignin('base', 'pmsm_current_loop_keep_overrides', 0);
save(result_path, 'results');
fprintf('Saved %s\n', result_path);
end

function tests = local_test_table()
rpm_to_rad_s = 2.0 * pi / 60.0;

tests(1).name = 'id_step';
tests(1).description = 'Id step, Iq held at 0 A, SVPWM bypassed.';
tests(1).id_before = 0.0;
tests(1).id_after = 1.0;
tests(1).iq_before = 0.0;
tests(1).iq_after = 0.0;
tests(1).mech_speed_rad_s = 0.0;
tests(1).use_svpwm = 0;
tests(1).vbus = 24.0;
tests(1).stop_time = 0.05;

tests(2).name = 'iq_step';
tests(2).description = 'Iq step, Id held at 0 A, SVPWM bypassed.';
tests(2).id_before = 0.0;
tests(2).id_after = 0.0;
tests(2).iq_before = 0.0;
tests(2).iq_after = 1.0;
tests(2).mech_speed_rad_s = 0.0;
tests(2).use_svpwm = 0;
tests(2).vbus = 24.0;
tests(2).stop_time = 0.05;

tests(3).name = 'bemf_2000rpm';
tests(3).description = '2000 rpm back-EMF influence test, current loop only.';
tests(3).id_before = 0.0;
tests(3).id_after = 0.0;
tests(3).iq_before = 0.0;
tests(3).iq_after = 1.0;
tests(3).mech_speed_rad_s = 2000.0 * rpm_to_rad_s;
tests(3).use_svpwm = 0;
tests(3).vbus = 24.0;
tests(3).stop_time = 0.05;

tests(4).name = 'voltage_limit_3000rpm';
tests(4).description = '3000 rpm voltage limit stress test.';
tests(4).id_before = 0.0;
tests(4).id_after = 0.0;
tests(4).iq_before = 0.0;
tests(4).iq_after = 5.0;
tests(4).mech_speed_rad_s = 3000.0 * rpm_to_rad_s;
tests(4).use_svpwm = 0;
tests(4).vbus = 24.0;
tests(4).stop_time = 0.05;
end

function summary = local_summarize(log)
% Column map:
% 1 id_ref, 2 iq_ref, 3 id, 4 iq, 5 vd_pi, 6 vq_pi,
% 7 vd_cmd, 8 vq_cmd, 9 limit_flag, 10 id_int, 11 iq_int,
% 12 valpha, 13 vbeta, 14 theta_e, 15 we,
% 16 duty_u, 17 duty_v, 18 duty_w, 19 sector.
summary.id_final = log(end, 3);
summary.iq_final = log(end, 4);
summary.id_ref_final = log(end, 1);
summary.iq_ref_final = log(end, 2);
summary.max_abs_id = max(abs(log(:, 3)));
summary.max_abs_iq = max(abs(log(:, 4)));
summary.max_vdq = max(sqrt((log(:, 7) .* log(:, 7)) + (log(:, 8) .* log(:, 8))));
summary.limit_seen = uint8(any(log(:, 9) > 0.5));
summary.max_abs_id_int = max(abs(log(:, 10)));
summary.max_abs_iq_int = max(abs(log(:, 11)));
summary.final_theta_e = log(end, 14);
summary.final_we = log(end, 15);
end
