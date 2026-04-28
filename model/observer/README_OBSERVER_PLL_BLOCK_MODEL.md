# PMSM Observer PLL Block Model

## Model Purpose

`pmsm_observer_pll_block_model.slx` is a graphical Simulink principle model for the current STM32G474 FOC project's Flux Observer + PLL path.

It is different from the code-like model in `model/observer/pmsm_observer_pll_discrete.slx`.

- The code-like model is for checking whether the Simulink implementation matches the C code path.
- The block-based model is for understanding the observer and PLL math structure visually.

This model does not include Handover, speed loop, startup, real PWM, dynamic ADC sampling, or Simscape power bridge.

## Top-Level Structure

The model is split into these visible top-level subsystems:

- `Test_Stimulus`
- `PMSM_SteadyState_Signal_Generator`
- `Flux_Observer`
- `PLL`
- `Error_Calculation`
- `Scopes_And_Logging`

## Flux Observer Subsystem

`Flux_Observer` is built with native Simulink blocks.

Main equations:

```text
e_alpha = valpha - Rs * ialpha
e_beta  = vbeta  - Rs * ibeta

flux_s_alpha = integral(e_alpha)
flux_s_beta  = integral(e_beta)

flux_r_alpha = flux_s_alpha - Ls * ialpha
flux_r_beta  = flux_s_beta  - Ls * ibeta

flux_mag = sqrt(flux_r_alpha^2 + flux_r_beta^2)
```

The main integration path uses `Gain`, `Sum`, and `Discrete-Time Integrator`.

The flux limiter is visible as a separate branch:

```text
if flux_mag > flux_limit:
    scale = flux_limit / flux_mag
else:
    scale = 1
```

This block-based limiter scales the visible flux outputs. It does not feed the scaled flux back into the internal integrator state exactly like the C code does. That is one known difference versus `foc_flux_observer_pll.c`.

## PLL Subsystem

`PLL` is also built from native Simulink blocks.

Main equations:

```text
sin_theta = sin(theta_raw)
cos_theta = cos(theta_raw)

pll_err_raw = flux_beta * cos_theta - flux_alpha * sin_theta
pll_err = pll_err_raw / max(flux_mag, eps)

pll_integrator = saturate(pll_integrator + pll_ki * pll_err * Ts)
speed_raw = saturate(pll_integrator + pll_kp * pll_err)
speed_obs = speed_obs_z1 + alpha * (speed_raw - speed_obs_z1)

theta_raw = wrap_to_2pi(theta_raw_z1 + speed_raw * Ts)
theta_comp = wrap_to_2pi(theta_raw + obs_theta_comp_rad)
```

The PLL uses:

- `Trigonometric Function` for sin/cos;
- `Product` for multiplications;
- `Sum` for additions/subtractions;
- `Divide` for normalized error;
- `Unit Delay` plus `Sum` and `Gain(Ts)` for discrete accumulation;
- `Saturation` for integral and speed limits;
- small MATLAB Function blocks only for `wrap_to_2pi`.

## PMSM Steady-State Signal Generator

The generator creates ideal alpha/beta voltage and current signals:

```text
we = pole_pairs * mech_speed_rad_s
theta_real = wrap_to_2pi(integral(we))

vd = Rs * id - we * Lq * iq
vq = Rs * iq + we * Ld * id + we * flux

valpha = vd * cos(theta_real) - vq * sin(theta_real)
vbeta  = vd * sin(theta_real) + vq * cos(theta_real)

ialpha = id * cos(theta_real) - iq * sin(theta_real)
ibeta  = id * sin(theta_real) + iq * cos(theta_real)
```

Most of this subsystem is native blocks. The only MATLAB Function block is the angle wrap.

## Scopes

The model contains separate scopes:

- `Scope_Angle`: `theta_real`, `theta_raw`, `theta_comp`
- `Scope_Speed`: `speed_real`, `speed_raw`, `speed_obs`
- `Scope_Error`: `theta_err_raw_deg`, `theta_err_comp_deg`, `speed_err_rad_s`
- `Scope_Flux`: `flux_alpha`, `flux_beta`, `flux_mag`
- `Scope_PLL`: `pll_err`, `pll_integrator`

The model also writes `observer_pll_block_log` to workspace.

## Log Columns

`observer_pll_block_log` columns:

1. `valpha`
2. `vbeta`
3. `ialpha`
4. `ibeta`
5. `theta_real`
6. `speed_real`
7. `flux_alpha`
8. `flux_beta`
9. `flux_mag`
10. `pll_err`
11. `pll_integrator`
12. `speed_raw`
13. `speed_obs`
14. `theta_raw`
15. `theta_comp`
16. `theta_err_raw_deg`
17. `theta_err_comp_deg`
18. `speed_err_rad_s`
19. `flux_limit_hit`
20. `speed_limit_hit`
21. `integral_limit_hit`

## Run

```matlab
addpath('D:/final_projects/G474_Sensorless_FOC/model/simulink');
build_pmsm_observer_pll_block_model;
out = sim('pmsm_observer_pll_block_model');
```

## Tests

```matlab
addpath('D:/final_projects/G474_Sensorless_FOC/model/simulink');
results = run_observer_pll_block_model_tests;
```

The test script runs:

- `observer_500rpm`
- `observer_1000rpm`
- `observer_2000rpm`
- `observer_reverse_500rpm`

Results are saved to:

```text
model/simulink/observer_pll_block_model_test_results.mat
```

## Current Differences From C

The model is intentionally graphical, so a few pieces are approximate:

- flux limit output is block-based and visible, but it does not write the limited flux back into the internal integrator state exactly like the C code;
- theta integration uses `Unit Delay + Sum + Gain(Ts)` style in the PLL, not static C variables;
- the test stimulus is steady-state PMSM alpha/beta generation, not a dynamic motor plant;
- no ADC sampling, PWM ripple, deadtime, Handover, speed loop, or startup transient is included.

Use the code-like model when exact C-path comparison matters. Use this block model when you want to inspect and explain the observer and PLL principle.
