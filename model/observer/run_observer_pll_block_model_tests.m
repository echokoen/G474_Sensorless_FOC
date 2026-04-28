function results = run_observer_pll_block_model_tests()
%RUN_OBSERVER_PLL_BLOCK_MODEL_TESTS Run block-based Observer/PLL model tests.

script_dir = fileparts(mfilename('fullpath'));
model_name = 'pmsm_observer_pll_block_model';
model_path = fullfile(script_dir, [model_name '.slx']);
param_path = fullfile(script_dir, 'pmsm_observer_pll_params.m');
result_path = fullfile(script_dir, 'observer_pll_block_model_test_results.mat');

if ~exist(model_path, 'file')
    build_pmsm_observer_pll_block_model;
end

evalin('base', sprintf('run(''%s'');', strrep(param_path, '''', '''''')));
load_system(model_path);

tests = local_test_table();
results = struct([]);

for k = 1:numel(tests)
    t = tests(k);
    evalin('base', sprintf('run(''%s'');', strrep(param_path, '''', '''''')));
    assignin('base', 'pmsm_observer_pll_keep_overrides', 1);
    assignin('base', 'sim_mech_speed_rad_s', t.mech_speed_rad_s);
    assignin('base', 'sim_id_a', t.id_a);
    assignin('base', 'sim_iq_a', t.iq_a);

    out = sim(model_name, ...
        'StopTime', num2str(t.stop_time), ...
        'ReturnWorkspaceOutputs', 'on');

    log = out.get('observer_pll_block_log');
    summary = local_summarize(log);

    results(k).name = t.name;
    results(k).description = t.description;
    results(k).settings = t;
    results(k).summary = summary;
    results(k).log = log;

    fprintf('%s: raw_angle_err=%.2f deg, comp_angle_err=%.2f deg, speed_err=%.2f rad/s, flux=%.5f Wb\n', ...
        t.name, summary.theta_raw_err_final_deg, summary.theta_comp_err_final_deg, ...
        summary.speed_err_final_rad_s, summary.flux_mag_final);
end

assignin('base', 'pmsm_observer_pll_keep_overrides', 0);
save(result_path, 'results');
fprintf('Saved %s\n', result_path);
end

function tests = local_test_table()
rpm_to_rad_s = 2.0 * pi / 60.0;

tests(1).name = 'observer_500rpm';
tests(1).description = 'Block model observer tracking at 500 rpm.';
tests(1).mech_speed_rad_s = 500.0 * rpm_to_rad_s;
tests(1).id_a = 0.0;
tests(1).iq_a = 1.0;
tests(1).stop_time = 0.15;

tests(2).name = 'observer_1000rpm';
tests(2).description = 'Block model observer tracking at 1000 rpm.';
tests(2).mech_speed_rad_s = 1000.0 * rpm_to_rad_s;
tests(2).id_a = 0.0;
tests(2).iq_a = 1.0;
tests(2).stop_time = 0.15;

tests(3).name = 'observer_2000rpm';
tests(3).description = 'Block model observer tracking at 2000 rpm.';
tests(3).mech_speed_rad_s = 2000.0 * rpm_to_rad_s;
tests(3).id_a = 0.0;
tests(3).iq_a = 1.0;
tests(3).stop_time = 0.15;

tests(4).name = 'observer_reverse_500rpm';
tests(4).description = 'Block model reverse direction check at -500 rpm.';
tests(4).mech_speed_rad_s = -500.0 * rpm_to_rad_s;
tests(4).id_a = 0.0;
tests(4).iq_a = 1.0;
tests(4).stop_time = 0.15;
end

function summary = local_summarize(log)
% Column map:
% 1 valpha, 2 vbeta, 3 ialpha, 4 ibeta, 5 theta_real, 6 speed_real,
% 7 flux_alpha, 8 flux_beta, 9 flux_mag, 10 pll_err, 11 pll_integrator,
% 12 speed_raw, 13 speed_obs, 14 theta_raw, 15 theta_comp,
% 16 theta_err_raw_deg, 17 theta_err_comp_deg, 18 speed_err_rad_s,
% 19 flux_limit_hit, 20 speed_limit_hit, 21 integral_limit_hit.
summary.theta_raw_err_final_deg = log(end, 16);
summary.theta_comp_err_final_deg = log(end, 17);
summary.speed_err_final_rad_s = log(end, 18);
summary.flux_mag_final = log(end, 9);
summary.max_abs_pll_err = max(abs(log(:, 10)));
summary.max_abs_speed_err_rad_s = max(abs(log(:, 18)));
summary.flux_limit_seen = uint8(any(log(:, 19) > 0.5));
summary.speed_limit_seen = uint8(any(log(:, 20) > 0.5));
summary.integral_limit_seen = uint8(any(log(:, 21) > 0.5));
end
