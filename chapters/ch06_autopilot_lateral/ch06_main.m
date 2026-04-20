% =========================================================================
%  CH06_MAIN - Chapter 6: PID Autopilot Design & Simulation
% =========================================================================
%  Designs PID gains using transfer function coefficients from Ch.5,
%  then runs a closed-loop nonlinear simulation to verify autopilot
%  performance across three command sequences:
%
%  Simulation timeline:
%    t = 0  s : Start at trim (Va=25, h=100m, chi=0°)
%    t = 5  s : Step Va_ref: 25 → 28 m/s
%    t = 25 s : Step h_ref:  100 → 130 m
%    t = 45 s : Step chi_ref: 0° → 45°
%
%  Test sections:
%    Section 1 - PID gain computation and sign/magnitude checks
%    Section 2 - Closed-loop nonlinear simulation (60 seconds)
%    Section 3 - Response analysis (rise time, overshoot, steady-state)
%
%  Architecture context:
%    Ch5 produced: x_trim, u_trim, tf_s  (trim + linearization)
%    Ch6 uses them to compute gains and simulate closed-loop behavior
%    Ch7 will add sensor noise and state estimation
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 6
% =========================================================================

% --- Auto-path setup (pwd-based, avoids Mac Editor temp-path bug) ---
atlas_root = fullfile(pwd, '..', '..');
addpath(atlas_root);
setup_paths(atlas_root);

clear; clc;
atlas_root = fullfile(pwd, '..', '..');
addpath(atlas_root);
setup_paths(atlas_root);

fprintf('\n========================================\n');
fprintf('  AtlasFC  --  Chapter 6: PID Autopilot\n');
fprintf('========================================\n\n');

params = mav_params();

% =========================================================================
%  PREREQUISITE: Load trim data saved by Ch.5
% =========================================================================
params_dir = fileparts(which('mav_params'));
trim_file  = fullfile(params_dir, 'trim_data.mat');
if ~exist(trim_file, 'file')
    error('trim_data.mat not found. Run ch05_main.m first to generate it.');
end
load(trim_file, 'x_trim', 'u_trim', 'tf_s', 'gains', 'Va_star');

fprintf('Trim loaded: Va*=%.1f m/s, alpha*=%.2f deg, delta_t*=%.1f%%\n', ...
    tf_s.Va_star, rad2deg(tf_s.alpha_star), tf_s.delta_t_star*100);

% =========================================================================
%  SECTION 1 — PID Gain Summary (gains loaded from trim_data.mat)
% =========================================================================
fprintf('\n--- SECTION 1: PID Gain Design ---\n');
% gains already loaded from trim_data.mat above

fprintf('\n  ROLL HOLD (phi/delta_a):\n');
fprintf('    Target: wn=%.1f rad/s, zeta=%.2f\n', gains.wn_phi, gains.zeta_phi);
fprintf('    kp_phi = %+.4f\n', gains.kp_phi);
fprintf('    kd_phi = %+.4f\n', gains.kd_phi);
% Verify closed-loop poles
wn_phi_cl   = sqrt(tf_s.a_phi2 * gains.kp_phi);
zeta_phi_cl = (tf_s.a_phi1 - tf_s.a_phi2*gains.kd_phi) / (2*wn_phi_cl);
fprintf('    Achieved: wn=%.2f rad/s, zeta=%.3f\n', wn_phi_cl, zeta_phi_cl);
if wn_phi_cl > 0 && zeta_phi_cl > 0
    fprintf('    [PASS] Stable roll hold.\n');
else
    fprintf('    [WARN] Check roll gains.\n');
end

fprintf('\n  PITCH HOLD (theta/delta_e):\n');
fprintf('    Target: wn=%.1f rad/s, zeta=%.2f\n', gains.wn_theta, gains.zeta_theta);
fprintf('    kp_theta = %+.4f  (negative is correct for a_theta3<0)\n', gains.kp_theta);
fprintf('    kd_theta = %+.4f\n', gains.kd_theta);
wn_theta_cl   = sqrt(tf_s.a_theta2 - tf_s.a_theta3*gains.kp_theta);
zeta_theta_cl = (tf_s.a_theta1 - tf_s.a_theta3*gains.kd_theta) / (2*wn_theta_cl);
fprintf('    Achieved: wn=%.2f rad/s, zeta=%.3f\n', wn_theta_cl, zeta_theta_cl);
if wn_theta_cl > 0 && zeta_theta_cl > 0
    fprintf('    [PASS] Stable pitch hold.\n');
else
    fprintf('    [WARN] Check pitch gains.\n');
end

fprintf('\n  AIRSPEED HOLD (Va/delta_t):\n');
fprintf('    Target: wn=%.1f rad/s, zeta=%.2f\n', gains.wn_V, gains.zeta_V);
fprintf('    kp_V = %+.4f\n', gains.kp_V);
fprintf('    ki_V = %+.4f\n', gains.ki_V);
wn_V_cl   = sqrt(tf_s.a_V2 * gains.ki_V);
zeta_V_cl = (tf_s.a_V1 + tf_s.a_V2*gains.kp_V) / (2*wn_V_cl);
fprintf('    Achieved: wn=%.2f rad/s, zeta=%.3f\n', wn_V_cl, zeta_V_cl);
if wn_V_cl > 0 && zeta_V_cl > 0
    fprintf('    [PASS] Stable airspeed hold.\n');
else
    fprintf('    [WARN] Check airspeed gains.\n');
end

fprintf('\n  ALTITUDE HOLD (h/theta_ref):\n');
fprintf('    kp_h = %.5f rad/m  (tau = %.1f s)\n', gains.kp_h, 1/(gains.Va_star*gains.kp_h));
fprintf('    zone = ±%.1f m -> full pitch = ±%.1f deg\n', gains.h_zone, rad2deg(gains.theta_max));

fprintf('\n  COURSE HOLD (chi/phi_ref):\n');
fprintf('    kp_chi = %.4f rad/rad\n', gains.kp_chi);
fprintf('    phi_max = %.1f deg\n', rad2deg(gains.phi_max));

% =========================================================================
%  SECTION 2 — Closed-Loop Nonlinear Simulation
% =========================================================================
fprintf('\n--- SECTION 2: Closed-Loop Simulation (60 s) ---\n');

dt   = 0.01;           % time step [s]
T    = 60.0;           % total time [s]
N    = round(T/dt);

% --- Command schedule ---
Va_init = 25.0;   Va_step = 28.0;   t_Va  = 5.0;
h_init  = 100.0;  h_step  = 130.0;  t_h   = 25.0;
chi_init = 0.0;   chi_step = deg2rad(45);  t_chi = 45.0;

% --- Initial state (at trim) ---
x12 = x_trim;

% --- Autopilot integrator state ---
ap_state.Va_int = 0;

% --- Pre-allocate history ---
t_hist    = zeros(1, N);
Va_hist   = zeros(1, N);
h_hist    = zeros(1, N);
chi_hist  = zeros(1, N);
theta_hist = zeros(1, N);
phi_hist  = zeros(1, N);
de_hist   = zeros(1, N);
da_hist   = zeros(1, N);
dt_hist   = zeros(1, N);
Va_cmd_hist  = zeros(1, N);
h_cmd_hist   = zeros(1, N);
chi_cmd_hist = zeros(1, N);

fprintf('  Running simulation...\n');
t0 = tic;

for k = 1:N
    t_now = (k-1) * dt;

    % --- Commands (step sequence) ---
    if t_now < t_Va
        cmd.Va_c  = Va_init;
    else
        cmd.Va_c  = Va_step;
    end
    if t_now < t_h
        cmd.h_c   = h_init;
    else
        cmd.h_c   = h_step;
    end
    if t_now < t_chi
        cmd.chi_c = chi_init;
    else
        cmd.chi_c = chi_step;
    end

    % --- Extract observables from state ---
    Va  = sqrt(x12(4)^2 + x12(5)^2 + x12(6)^2);
    h   = -x12(3);
    chi = x12(9);

    % --- Record ---
    t_hist(k)    = t_now;
    Va_hist(k)   = Va;
    h_hist(k)    = h;
    chi_hist(k)  = chi;
    theta_hist(k) = x12(8);
    phi_hist(k)  = x12(7);
    Va_cmd_hist(k)  = cmd.Va_c;
    h_cmd_hist(k)   = cmd.h_c;
    chi_cmd_hist(k) = cmd.chi_c;

    % --- Autopilot ---
    [u_cmd, ap_state] = autopilot(cmd, x12, u_trim, gains, ap_state, dt);

    de_hist(k)  = u_cmd(1);
    da_hist(k)  = u_cmd(2);
    dt_hist(k)  = u_cmd(4);

    % --- Propagate nonlinear dynamics ---
    e_q = euler_to_quaternion(x12(7), x12(8), x12(9));
    x13 = [x12(1:6); e_q; x12(10:12)];

    [fm, ~, ~, ~] = forces_moments(x13, u_cmd, zeros(3,1), zeros(3,1), params);
    x13_new = mav_dynamics(x13, fm, params, dt);

    % Convert back to 12-state (Euler)
    [phi_n, theta_n, psi_n] = quaternion_to_euler(x13_new(7:10));
    x12 = [x13_new(1:3); x13_new(4:6); phi_n; theta_n; psi_n; x13_new(11:13)];
end

sim_time = toc(t0);
fprintf('  Simulation complete in %.2f s (%.0f steps).\n', sim_time, N);

% =========================================================================
%  SECTION 3 — Response Analysis
% =========================================================================
fprintf('\n--- SECTION 3: Response Analysis ---\n');

% Airspeed response (after step at t=5s)
idx_Va   = t_hist >= t_Va;
Va_final = mean(Va_hist(t_hist >= 55));
Va_err   = abs(Va_final - Va_step);
fprintf('\n  AIRSPEED step (%.1f → %.1f m/s):\n', Va_init, Va_step);
fprintf('    Final Va     = %.2f m/s\n', Va_final);
fprintf('    Steady error = %.3f m/s\n', Va_err);
if Va_err < 0.5
    fprintf('    [PASS] Va steady-state error < 0.5 m/s.\n');
else
    fprintf('    [WARN] Va steady-state error = %.3f m/s.\n', Va_err);
end

% Altitude response (after step at t=25s)
h_final = mean(h_hist(t_hist >= 55));
h_err   = abs(h_final - h_step);
fprintf('\n  ALTITUDE step (%.0f → %.0f m):\n', h_init, h_step);
fprintf('    Final h      = %.2f m\n', h_final);
fprintf('    Steady error = %.2f m\n', h_err);
if h_err < 2.0
    fprintf('    [PASS] Altitude steady-state error < 2 m.\n');
else
    fprintf('    [WARN] Altitude steady-state error = %.2f m.\n', h_err);
end

% Course response (after step at t=45s)
chi_final = mean(chi_hist(t_hist >= 58));
chi_err   = abs(rad2deg(chi_final - chi_step));
fprintf('\n  COURSE step (0 → 45 deg):\n');
fprintf('    Final chi    = %.2f deg\n', rad2deg(chi_final));
fprintf('    Steady error = %.2f deg\n', chi_err);
if chi_err < 3.0
    fprintf('    [PASS] Course steady-state error < 3 deg.\n');
else
    fprintf('    [WARN] Course steady-state error = %.2f deg.\n', chi_err);
end

% =========================================================================
%  PLOTS
% =========================================================================

% --- Plot 1: Airspeed response ---
figure('Name', 'Ch06: Airspeed Hold', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t_hist, Va_hist, 'b', 'LineWidth', 1.5); hold on;
stairs(t_hist, Va_cmd_hist, 'r--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Va [m/s]');
title('Airspeed Hold — Va response');
legend('Va (actual)', 'Va (commanded)'); grid on;

subplot(2,1,2);
plot(t_hist, dt_hist, 'g', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\delta_t [-]');
title('Throttle command'); grid on;

% --- Plot 2: Altitude and Pitch ---
figure('Name', 'Ch06: Altitude Hold', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t_hist, h_hist, 'b', 'LineWidth', 1.5); hold on;
stairs(t_hist, h_cmd_hist, 'r--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('h [m]');
title('Altitude Hold — h response');
legend('h (actual)', 'h (commanded)'); grid on;

subplot(2,1,2);
plot(t_hist, rad2deg(theta_hist), 'g', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\theta [deg]');
title('Pitch angle (commanded by altitude hold)'); grid on;

% --- Plot 3: Course and Roll ---
figure('Name', 'Ch06: Course Hold', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t_hist, rad2deg(chi_hist), 'b', 'LineWidth', 1.5); hold on;
stairs(t_hist, rad2deg(chi_cmd_hist), 'r--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('\chi [deg]');
title('Course Hold — \chi response');
legend('\chi (actual)', '\chi (commanded)'); grid on;

subplot(2,1,2);
plot(t_hist, rad2deg(phi_hist), 'g', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\phi [deg]');
title('Roll angle (commanded by course hold)'); grid on;

% --- Plot 4: Control surfaces ---
figure('Name', 'Ch06: Control Inputs', 'NumberTitle', 'off');
subplot(3,1,1);
plot(t_hist, rad2deg(de_hist), 'b', 'LineWidth', 1.5);
ylabel('\delta_e [deg]'); title('Elevator'); grid on;

subplot(3,1,2);
plot(t_hist, rad2deg(da_hist), 'r', 'LineWidth', 1.5);
ylabel('\delta_a [deg]'); title('Aileron'); grid on;

subplot(3,1,3);
plot(t_hist, dt_hist, 'g', 'LineWidth', 1.5);
ylabel('\delta_t [-]'); title('Throttle');
xlabel('Time [s]'); grid on;

fprintf('\n========================================\n');
fprintf('  Chapter 6 complete. 4 plot windows.\n');
fprintf('========================================\n\n');
fprintf('  PID Gain Summary:\n');
fprintf('    Roll:     kp=%.4f  kd=%.4f\n', gains.kp_phi, gains.kd_phi);
fprintf('    Pitch:    kp=%.4f  kd=%.4f\n', gains.kp_theta, gains.kd_theta);
fprintf('    Airspeed: kp=%.4f  ki=%.4f\n', gains.kp_V, gains.ki_V);
fprintf('    Altitude: kp=%.5f [rad/m]\n',  gains.kp_h);
fprintf('    Course:   kp=%.4f [rad/rad]\n', gains.kp_chi);
