% PMSM observer + PLL block model parameters.
%
% Values are copied from the current C project:
% Config/foc_config.h
% Control/Observer/foc_observer.c
% Control/Model/foc_flux_observer_pll.c

if exist('pmsm_observer_pll_keep_overrides', 'var') && ...
        (pmsm_observer_pll_keep_overrides ~= 0)
    return;
end

Ts = 6.25e-5;

Rs = 0.53;
Ls = 0.0006;
Ld = Ls;
Lq = Ls;
flux_wb = 0.0078;
flux_limit_ratio = 1.50;
flux_limit = flux_wb * flux_limit_ratio;
pole_pairs = 4.0;

obs_theta_comp_rad = 2.2003;
pll_kp = 120.0;
pll_ki = 3000.0;
pll_integral_limit = 1800.0;
obs_speed_limit_rad_s = 2200.0;
obs_speed_filter_alpha = 0.05;
obs_eps = 1.0e-6;

sim_mech_speed_rad_s = 2000.0 * 2.0 * pi / 60.0;
sim_id_a = 0.0;
sim_iq_a = 1.0;
