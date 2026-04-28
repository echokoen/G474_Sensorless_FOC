function results = run_pwm_adc_sampling_timing_tests()
%RUN_PWM_ADC_SAMPLING_TIMING_TESTS Verify PWM/ADC dynamic sampling timing.

script_dir = fileparts(mfilename('fullpath'));
param_path = fullfile(script_dir, 'pwm_adc_sampling_params.m');
figure_dir = fullfile(script_dir, 'figures');
result_path = fullfile(script_dir, 'pwm_adc_sampling_test_results.mat');

if ~exist(figure_dir, 'dir')
    mkdir(figure_dir);
end

run(param_path);

cases = struct([]);
for k = 1:numel(single_test_modulation)
    m = single_test_modulation(k);
    cases(k).name = sprintf('mod_%.2f', m);
    cases(k).modulation_index = m;
    cases(k).table = local_sweep_one_modulation(m, theta_vec, vbus);
    cases(k).summary = local_summary(cases(k).table);
end

full_sweep = local_sweep_grid(modulation_index_list, theta_vec, vbus);
full_summary = local_grid_summary(full_sweep);

results.cases = cases;
results.full_sweep = full_sweep;
results.full_summary = full_summary;

local_plot_single_case(cases(1).table, figure_dir, 'low_mod_0p30');
local_plot_single_case(cases(2).table, figure_dir, 'mid_mod_0p60');
local_plot_single_case(cases(3).table, figure_dir, 'high_mod_0p90');
local_plot_heatmap(full_sweep, figure_dir);

save(result_path, 'results');

fprintf('PWM/ADC sampling timing tests saved: %s\n', result_path);
fprintf('Full sweep: total=%u valid=%u fallback=%u fallback_ratio=%.4f min_safe_width=%g worst_angle_deg=%.2f worst_modulation=%.2f\n', ...
    full_summary.total_points, full_summary.valid_points, full_summary.fallback_points, ...
    full_summary.fallback_ratio, full_summary.min_safe_width, ...
    full_summary.worst_angle_deg, full_summary.worst_modulation);
end

function table_out = local_sweep_one_modulation(modulation_index, theta_vec, vbus)
n = numel(theta_vec);
table_out = local_alloc_table(n);

for i = 1:n
    theta = theta_vec(i);
    vref = modulation_index * vbus / sqrt(3.0);
    valpha = vref * cos(theta);
    vbeta = vref * sin(theta);
    row = local_eval_point(valpha, vbeta, vbus, modulation_index, theta);
    table_out = local_store_row(table_out, i, row);
end
end

function grid = local_sweep_grid(modulation_list, theta_vec, vbus)
rows = numel(modulation_list) * numel(theta_vec);
grid = local_alloc_table(rows);
idx = 1;

for m = modulation_list
    one = local_sweep_one_modulation(m, theta_vec, vbus);
    count = numel(one.theta_rad);
    fields = fieldnames(one);
    for f = 1:numel(fields)
        grid.(fields{f})(idx:(idx + count - 1), 1) = one.(fields{f})(:);
    end
    idx = idx + count;
end
end

function row = local_eval_point(valpha, vbeta, vbus, modulation_index, theta)
sector = local_voltage_sector(valpha, vbeta);
[t1, t2, t0] = local_sector_times(valpha, vbeta, vbus, sector);
[duty_u, duty_v, duty_w, ccr_u, ccr_v, ccr_w] = local_duty_ccr(sector, t1, t2, t0);
[trusted_pair, trusted_phase_a, trusted_phase_b, reconstruct_phase, ccr_x, ccr_y] = ...
    local_trusted_pair(sector, ccr_u, ccr_v, ccr_w);
[win_start, win_end, win_width, adc_sample_ccr, fallback_flag, win1, win2, midpoint_sampling, dynamic_sampling] = ...
    local_sampling_window(ccr_x, ccr_y);

hold_last_flag = fallback_flag && (local_param('adc_hold_last_on_fallback') ~= 0);
valid_sample_flag = ~fallback_flag;

row = struct();
row.theta_rad = theta;
row.theta_deg = theta * 180.0 / pi;
row.modulation_index = modulation_index;
row.valpha = valpha;
row.vbeta = vbeta;
row.sector = sector;
row.t1 = t1;
row.t2 = t2;
row.t0 = t0;
row.duty_u = duty_u;
row.duty_v = duty_v;
row.duty_w = duty_w;
row.ccr_u = ccr_u;
row.ccr_v = ccr_v;
row.ccr_w = ccr_w;
row.trusted_pair = trusted_pair;
row.trusted_phase_a = trusted_phase_a;
row.trusted_phase_b = trusted_phase_b;
row.reconstruct_phase = reconstruct_phase;
row.win_start = win_start;
row.win_end = win_end;
row.win_width = win_width;
row.adc_sample_ccr = adc_sample_ccr;
row.fallback_flag = double(fallback_flag);
row.hold_last_flag = double(hold_last_flag);
row.valid_sample_flag = double(valid_sample_flag);
row.win1 = win1;
row.win2 = win2;
row.midpoint_sampling = double(midpoint_sampling);
row.dynamic_sampling = double(dynamic_sampling);
end

function sector = local_voltage_sector(valpha, vbeta)
two_pi = 6.2831853071795864769;
inv_sector_width = 0.95492965855;
angle = atan2(vbeta, valpha);
if angle < 0.0
    angle = angle + two_pi;
end
sector = floor(angle * inv_sector_width) + 1;
if sector > 6
    sector = 6;
end
end

function [t1, t2, t0] = local_sector_times(valpha, vbeta, vbus, sector)
two_pi = 6.2831853071795864769;
pi_over_3 = 1.0471975511965977462;
sqrt3 = 1.7320508075688772;
pwm_period = local_param('pwm_arr');
norm_vbus = vbus;
if norm_vbus <= 0.1
    norm_vbus = 24.0;
end

angle = atan2(vbeta, valpha);
if angle < 0.0
    angle = angle + two_pi;
end

theta_sector = angle - ((sector - 1) * pi_over_3);
theta_sector = min(max(theta_sector, 0.0), pi_over_3);

vref = sqrt((valpha * valpha) + (vbeta * vbeta));
t1 = pwm_period * sqrt3 * (vref / norm_vbus) * sin(pi_over_3 - theta_sector);
t2 = pwm_period * sqrt3 * (vref / norm_vbus) * sin(theta_sector);
t1 = max(t1, 0.0);
t2 = max(t2, 0.0);

if (t1 + t2) > pwm_period
    scale = pwm_period / (t1 + t2);
    t1 = t1 * scale;
    t2 = t2 * scale;
end

t0 = pwm_period - t1 - t2;
t0 = max(t0, 0.0);
end

function [du, dv, dw, ccr_u, ccr_v, ccr_w] = local_duty_ccr(sector, t1, t2, t0)
pwm_period = local_param('pwm_arr');
half_t0 = 0.5 * t0;

switch sector
    case 1
        ta = t1 + t2 + half_t0; tb = t2 + half_t0;      tc = half_t0;
    case 2
        ta = t1 + half_t0;      tb = t1 + t2 + half_t0; tc = half_t0;
    case 3
        ta = half_t0;           tb = t1 + t2 + half_t0; tc = t2 + half_t0;
    case 4
        ta = half_t0;           tb = t1 + half_t0;      tc = t1 + t2 + half_t0;
    case 5
        ta = t2 + half_t0;      tb = half_t0;           tc = t1 + t2 + half_t0;
    otherwise
        ta = t1 + t2 + half_t0; tb = half_t0;           tc = t1 + half_t0;
end

du = min(max(ta / pwm_period, 0.0), 1.0);
dv = min(max(tb / pwm_period, 0.0), 1.0);
dw = min(max(tc / pwm_period, 0.0), 1.0);

ccr_u = local_clamp_ccr(floor(du * pwm_period));
ccr_v = local_clamp_ccr(floor(dv * pwm_period));
ccr_w = local_clamp_ccr(floor(dw * pwm_period));
end

function ccr = local_clamp_ccr(ccr)
ccr = min(max(ccr, local_param('svpwm_min_ccr')), local_param('svpwm_max_ccr'));
end

function [pair, phase_a, phase_b, reconstruct, ccr_x, ccr_y] = local_trusted_pair(sector, ccr_u, ccr_v, ccr_w)
switch sector
    case {1, 6}
        pair = 1; phase_a = 2; phase_b = 3; reconstruct = 1; ccr_x = ccr_v; ccr_y = ccr_w;
    case {2, 3}
        pair = 2; phase_a = 1; phase_b = 3; reconstruct = 2; ccr_x = ccr_u; ccr_y = ccr_w;
    otherwise
        pair = 3; phase_a = 1; phase_b = 2; reconstruct = 3; ccr_x = ccr_u; ccr_y = ccr_v;
end
end

function [safe_start, safe_end, safe_width, sample, fallback, win1, win2, midpoint_sampling, dynamic_sampling] = local_sampling_window(ccr_x, ccr_y)
pwm_arr = local_param('pwm_arr');
center = local_param('pwm_half_arr');
guard = local_param('adc_margin_ccr');
min_sample = local_param('adc_sample_min_ccr');
max_sample = local_param('adc_sample_max_ccr');
min_window = local_param('adc_min_window_ccr');
pair_max = max(ccr_x, ccr_y);

if pair_max > (pwm_arr - guard)
    safe_start = pwm_arr;
else
    safe_start = pair_max + guard;
end

safe_end = max_sample;
if safe_start < min_sample
    safe_start = min_sample;
end

if safe_start < safe_end
    safe_width = safe_end - safe_start;
else
    safe_width = 0;
end

if (local_param('adc_dynamic_sample_enable') ~= 0) && (safe_width >= min_window)
    mid = floor(safe_start + (safe_width / 2));
    if (center >= safe_start) && (center <= safe_end) && ...
            ((center - safe_start) >= (min_window / 2)) && ...
            ((safe_end - center) >= (min_window / 2))
        sample = center;
    else
        sample = mid;
    end
    fallback = false;
else
    sample = local_param('adc_fallback_ccr');
    fallback = true;
end

sample = local_clamp_ccr(sample);
midpoint_sampling = (sample == center);
dynamic_sampling = (sample ~= center);

if sample > safe_start
    win1 = sample - safe_start;
else
    win1 = 0;
end

if sample < safe_end
    win2 = safe_end - sample;
else
    win2 = 0;
end
end

function table_out = local_alloc_table(n)
fields = {'theta_rad','theta_deg','modulation_index','valpha','vbeta','sector','t1','t2','t0', ...
    'duty_u','duty_v','duty_w','ccr_u','ccr_v','ccr_w','trusted_pair','trusted_phase_a', ...
    'trusted_phase_b','reconstruct_phase','win_start','win_end','win_width','adc_sample_ccr', ...
    'fallback_flag','hold_last_flag','valid_sample_flag','win1','win2','midpoint_sampling','dynamic_sampling'};
for k = 1:numel(fields)
    table_out.(fields{k}) = zeros(n, 1);
end
end

function table_out = local_store_row(table_out, idx, row)
fields = fieldnames(row);
for k = 1:numel(fields)
    table_out.(fields{k})(idx, 1) = row.(fields{k});
end
end

function summary = local_summary(t)
summary.total_points = numel(t.theta_rad);
summary.valid_points = sum(t.valid_sample_flag > 0.5);
summary.fallback_points = sum(t.fallback_flag > 0.5);
summary.fallback_ratio = summary.fallback_points / summary.total_points;
summary.min_safe_width = min(t.win_width);
[~, idx] = min(t.win_width);
summary.worst_angle_deg = t.theta_deg(idx);
summary.worst_modulation = t.modulation_index(idx);
summary.max_sample_ccr = max(t.adc_sample_ccr);
summary.min_sample_ccr = min(t.adc_sample_ccr);
end

function summary = local_grid_summary(t)
summary = local_summary(t);
end

function local_plot_single_case(t, figure_dir, tag)
local_save_plot(figure_dir, ['pwm_adc_sector_vs_angle_' tag '.png'], @() local_plot_xy(t.theta_deg, t.sector, 'Angle deg', 'Sector', ['Sector vs angle ' tag]));
local_save_plot(figure_dir, ['pwm_adc_ccr_vs_angle_' tag '.png'], @() local_plot_multi(t.theta_deg, [t.ccr_u t.ccr_v t.ccr_w], 'Angle deg', 'CCR', {'U','V','W'}, ['CCR vs angle ' tag]));
local_save_plot(figure_dir, ['pwm_adc_sample_ccr_vs_angle_' tag '.png'], @() local_plot_xy(t.theta_deg, t.adc_sample_ccr, 'Angle deg', 'ADC sample CCR', ['ADC sample vs angle ' tag]));
local_save_plot(figure_dir, ['pwm_adc_safe_width_vs_angle_' tag '.png'], @() local_plot_xy(t.theta_deg, t.win_width, 'Angle deg', 'Safe width CCR', ['Safe width vs angle ' tag]));
local_save_plot(figure_dir, ['pwm_adc_fallback_vs_angle_' tag '.png'], @() local_plot_xy(t.theta_deg, t.fallback_flag, 'Angle deg', 'Fallback', ['Fallback vs angle ' tag]));
end

function local_plot_heatmap(t, figure_dir)
mods = unique(t.modulation_index);
angles = unique(t.theta_deg);
fallback = reshape(t.fallback_flag, numel(angles), numel(mods))';
local_save_plot(figure_dir, 'pwm_adc_fallback_heatmap.png', @() imagesc(angles, mods, fallback));
xlabel('Angle deg');
ylabel('Modulation index');
title('Fallback heatmap');
colorbar;
set(gca, 'YDir', 'normal');
end

function local_save_plot(figure_dir, name, plotter)
fig = figure('Visible', 'off');
plotter();
grid on;
saveas(fig, fullfile(figure_dir, name));
close(fig);
end

function local_plot_xy(x, y, xl, yl, ttl)
plot(x, y, 'LineWidth', 1.2);
xlabel(xl);
ylabel(yl);
title(ttl);
end

function local_plot_multi(x, y, xl, yl, legends, ttl)
plot(x, y, 'LineWidth', 1.2);
xlabel(xl);
ylabel(yl);
legend(legends, 'Location', 'best');
title(ttl);
end

function value = local_param(name)
persistent p
if isempty(p)
    p = struct();
    p.pwm_arr = 5311;
    p.pwm_half_arr = 2655;
    p.adc_margin_ccr = 140;
    p.adc_min_window_ccr = 320;
    p.adc_fallback_ccr = p.pwm_half_arr;
    p.adc_dynamic_sample_enable = 1;
    p.adc_hold_last_on_fallback = 1;
    p.adc_sample_min_ccr = p.adc_margin_ccr;
    p.adc_sample_max_ccr = p.pwm_arr - p.adc_margin_ccr;
    p.svpwm_min_ccr = 20;
    p.svpwm_max_ccr = p.pwm_arr - 20;
end
value = p.(name);
end
