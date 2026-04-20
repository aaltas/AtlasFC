% =========================================================================
%  CH08_MAIN - Chapter 8: INS/GPS EKF State Estimator
% =========================================================================
%  Runs a 60-second closed-loop simulation with the full pipeline:
%    Ch.7 sensors  →  Ch.8 EKF  →  Ch.6 autopilot
%
%  The autopilot now operates on ESTIMATED states from the EKF, not the
%  true simulation state.  This is the realistic scenario.
%
%  Command schedule (same as Ch.6):
%    t = 5  s : Va_ref: 25 → 28 m/s
%    t = 25 s : h_ref:  100 → 130 m
%    t = 45 s : chi_ref: 0° → 45°
%
%  Sections:
%    Section 1 — EKF parameter summary
%    Section 2 — Closed-loop simulation (60 s)
%    Section 3 — Estimation error analysis
%    Section 4 — Plots (true vs estimated, innovations)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 8
% =========================================================================

% --- Auto-path setup ---
atlas_root = fullfile(pwd, '..', '..');
addpath(atlas_root);
setup_paths(atlas_root);

clear; clc; rng(0);   % fixed seed for reproducible noise
atlas_root = fullfile(pwd, '..', '..');
addpath(atlas_root);
setup_paths(atlas_root);

fprintf('\n========================================\n');
fprintf('  AtlasFC  --  Chapter 8: EKF Estimator\n');
fprintf('========================================\n\n');

params  = mav_params();
sparams = sensor_params();
ep      = ekf_params(sparams);

% =========================================================================
%  SECTION 1 — Parameter summary
% =========================================================================
fprintf('--- EKF Parameters ---\n');
fprintf('  State: 13 (pos3, vel3, quat4, gyro_bias3)\n');
fprintf('  Propagation: 100 Hz  |  Process noise diagonal:\n');
fprintf('    Q_pos   = %.2e m^2\n',    ep.Q(1,1));
fprintf('    Q_vel   = %.2e (m/s)^2\n', ep.Q(4,4));
fprintf('    Q_att   = %.2e rad^2\n',   ep.Q(7,7));
fprintf('    Q_bias  = %.2e (rad/s)^2\n', ep.Q(11,11));
fprintf('  Sensor update rates: baro=%.0fHz, pitot=%.0fHz, mag=%.0fHz, GPS=%.0fHz\n\n', ...
    sparams.baro_hz, sparams.pitot_hz, sparams.mag_hz, sparams.gps_hz);

% =========================================================================
%  PREREQUISITE: Load trim data saved by Ch.5
% =========================================================================
params_dir = fileparts(which('mav_params'));
trim_file  = fullfile(params_dir, 'trim_data.mat');
if ~exist(trim_file, 'file')
    error('trim_data.mat not found. Run ch05_main.m first.');
end
load(trim_file, 'x_trim', 'u_trim', 'tf_s', 'gains', 'Va_star');

% Build 13-state trim
e_q_trim = euler_to_quaternion(x_trim(7), x_trim(8), x_trim(9));
x13_trim = [x_trim(1:6); e_q_trim; x_trim(10:12)];
x13_trim(1:3) = [0; 0; -100];   % h=100m

fprintf('Trim: Va*=%.1f m/s, h*=100m, alpha*=%.2f deg\n\n', ...
    Va_star, rad2deg(x_trim(8)));

% =========================================================================
%  SECTION 2 — Closed-loop simulation
% =========================================================================
fprintf('--- SECTION 2: Closed-loop Simulation (60 s) ---\n');

dt = 0.01;   T = 60.0;   N = round(T/dt);

% --- Command schedule ---
Va_init=25; Va_step=28; t_Va=5;
h_init=100; h_step=130; t_h=25;
chi_init=0; chi_step=deg2rad(45); t_chi=45;

% --- Initialize ---
x13          = x13_trim;
sensor_state = sensors_init(sparams, x13_trim);
[x_hat, P]   = ekf_init(x13_trim, ep);
ap_state.Va_int = 0;

% Add small random initial error to EKF state
x_hat(1:3)  = x_hat(1:3) + 2*randn(3,1);    % ±2m position error
x_hat(4:6)  = x_hat(4:6) + 0.3*randn(3,1);  % ±0.3 m/s velocity error
% (quaternion kept exact at init)

% --- Pre-allocate history ---
t_hist       = zeros(1, N);
% True states
Va_true_h    = zeros(1,N);  h_true_h  = zeros(1,N);  chi_true_h = zeros(1,N);
phi_true_h   = zeros(1,N);  theta_true_h = zeros(1,N);
p_true_h     = zeros(1,N);  q_true_h  = zeros(1,N);  r_true_h   = zeros(1,N);
bp_true_h    = zeros(1,N);  bq_true_h = zeros(1,N);  br_true_h  = zeros(1,N);
% Estimated states
Va_est_h     = zeros(1,N);  h_est_h   = zeros(1,N);  chi_est_h  = zeros(1,N);
phi_est_h    = zeros(1,N);  theta_est_h = zeros(1,N);
bp_est_h     = zeros(1,N);  bq_est_h  = zeros(1,N);  br_est_h   = zeros(1,N);
% Std devs (sqrt of diagonal P)
sig_h_h      = zeros(1,N);  sig_Va_h  = zeros(1,N);
sig_phi_h    = zeros(1,N);  sig_theta_h = zeros(1,N); sig_psi_h = zeros(1,N);
% Innovations
innov_baro_h = zeros(1,N);  innov_pitot_h = zeros(1,N);
innov_mag_h  = zeros(1,N);  innov_gps_h   = zeros(4,N);

fprintf('  Running simulation...\n');
t0_sim = tic;

for k = 1:N
    t_now = (k-1)*dt;
    t_hist(k) = t_now;

    % --- Commands ---
    cmd.Va_c  = Va_init  + (Va_step  - Va_init)  * (t_now >= t_Va);
    cmd.h_c   = h_init   + (h_step   - h_init)   * (t_now >= t_h);
    cmd.chi_c = chi_init + (chi_step - chi_init)  * (t_now >= t_chi);

    % --- True state observables ---
    R_true = quaternion_to_rotation(x13(7:10));
    [phi_t, theta_t, psi_t] = rotation_to_euler(R_true);
    Va_true_h(k)    = sqrt(x13(4)^2+x13(5)^2+x13(6)^2);
    h_true_h(k)     = -x13(3);
    chi_true_h(k)   = psi_t;
    phi_true_h(k)   = phi_t;
    theta_true_h(k) = theta_t;
    p_true_h(k)     = x13(11);
    q_true_h(k)     = x13(12);
    r_true_h(k)     = x13(13);

    % --- Compute forces at TRUE state (for sensor model) ---
    [fm, ~, ~, ~] = forces_moments(x13, u_trim, zeros(3,1), zeros(3,1), params);

    % --- Ch.7 Sensors ---
    [y, sensor_state] = sensors(x13, fm, sensor_state, sparams, params, dt);

    % --- Ch.8 EKF ---
    [x_hat, P, innov] = ekf(x_hat, P, y, ep, params, dt);

    % --- Record estimated states ---
    R_est = quaternion_to_rotation(x_hat(7:10));
    [phi_e, theta_e, psi_e] = rotation_to_euler(R_est);
    Va_est_h(k)     = sqrt(x_hat(4)^2+x_hat(5)^2+x_hat(6)^2);
    h_est_h(k)      = -x_hat(3);
    chi_est_h(k)    = psi_e;
    phi_est_h(k)    = phi_e;
    theta_est_h(k)  = theta_e;
    bp_est_h(k)     = x_hat(11);
    bq_est_h(k)     = x_hat(12);
    br_est_h(k)     = x_hat(13);

    % Uncertainty (1-sigma)
    sig_h_h(k)     = sqrt(P(3,3));
    sig_Va_h(k)    = sqrt((P(4,4)+P(5,5)+P(6,6))/3);
    sig_phi_h(k)   = sqrt(P(8,8)+P(9,9))*2;   % approx
    sig_theta_h(k) = sqrt(P(8,8)+P(9,9))*2;
    sig_psi_h(k)   = sqrt(P(9,9)+P(10,10))*2;

    % Innovations
    innov_baro_h(k)   = innov.baro;
    innov_pitot_h(k)  = innov.pitot;
    innov_mag_h(k)    = innov.mag;
    innov_gps_h(:,k)  = innov.gps;

    % --- Ch.6 Autopilot (using ESTIMATED states) ---
    % Build 12-state from EKF estimate
    x12_est = [x_hat(1:6); phi_e; theta_e; psi_e; x_hat(11:13)];
    % Use estimated angular rates from gyro (bias corrected)
    x12_est(10:12) = y.gyro - x_hat(11:13);

    [u_cmd, ap_state] = autopilot(cmd, x12_est, u_trim, gains, ap_state, dt);

    % --- Propagate TRUE nonlinear dynamics ---
    [fm_true, ~, ~, ~] = forces_moments(x13, u_cmd, zeros(3,1), zeros(3,1), params);
    x13 = mav_dynamics(x13, fm_true, params, dt);
end

sim_time = toc(t0_sim);
fprintf('  Done in %.2f s (%d steps).\n', sim_time, N);

% =========================================================================
%  SECTION 3 — Estimation error analysis
% =========================================================================
fprintf('\n--- SECTION 3: Estimation Errors (last 20s) ---\n');

idx = t_hist >= 40;

Va_err   = Va_est_h(idx)  - Va_true_h(idx);
h_err    = h_est_h(idx)   - h_true_h(idx);
chi_err  = atan2(sin(chi_est_h(idx)-chi_true_h(idx)), cos(chi_est_h(idx)-chi_true_h(idx)));
phi_err  = phi_est_h(idx) - phi_true_h(idx);
theta_err= theta_est_h(idx)-theta_true_h(idx);

fprintf('\n  Va error    : RMS=%.4f m/s   max=%.4f m/s\n', ...
    rms(Va_err),  max(abs(Va_err)));
fprintf('  h  error    : RMS=%.4f m     max=%.4f m\n', ...
    rms(h_err),   max(abs(h_err)));
fprintf('  chi error   : RMS=%.4f deg   max=%.4f deg\n', ...
    rad2deg(rms(chi_err)), rad2deg(max(abs(chi_err))));
fprintf('  phi error   : RMS=%.4f deg\n', rad2deg(rms(phi_err)));
fprintf('  theta error : RMS=%.4f deg\n', rad2deg(rms(theta_err)));

% Gyro bias convergence
fprintf('\n  Gyro bias estimate (end):\n');
fprintf('    bp_hat=%.5f rad/s  bq_hat=%.5f  br_hat=%.5f\n', ...
    x_hat(11), x_hat(12), x_hat(13));

% =========================================================================
%  SECTION 4 — Plots
% =========================================================================

% --- Plot 1: Altitude estimation ---
figure('Name','Ch08: Altitude','NumberTitle','off');
subplot(2,1,1);
plot(t_hist, h_true_h,'b','LineWidth',1.5); hold on;
plot(t_hist, h_est_h, 'r--','LineWidth',1.2);
fill([t_hist, fliplr(t_hist)], ...
     [h_est_h+3*sig_h_h, fliplr(h_est_h-3*sig_h_h)], ...
     'r','FaceAlpha',0.1,'EdgeColor','none');
ylabel('h [m]'); title('Altitude: true vs estimated (±3σ)');
legend('True','Estimated','3σ bound'); grid on;
subplot(2,1,2);
plot(t_hist, h_est_h - h_true_h,'k','LineWidth',1.0);
ylabel('Error [m]'); xlabel('Time [s]'); grid on;
title('Altitude estimation error');

% --- Plot 2: Airspeed estimation ---
figure('Name','Ch08: Airspeed','NumberTitle','off');
subplot(2,1,1);
plot(t_hist, Va_true_h,'b','LineWidth',1.5); hold on;
plot(t_hist, Va_est_h,'r--','LineWidth',1.2);
ylabel('Va [m/s]'); title('Airspeed: true vs estimated'); legend('True','Est'); grid on;
subplot(2,1,2);
plot(t_hist, Va_est_h - Va_true_h,'k','LineWidth',1.0);
ylabel('Error [m/s]'); xlabel('Time [s]'); grid on;

% --- Plot 3: Attitude estimation ---
figure('Name','Ch08: Attitude','NumberTitle','off');
subplot(3,1,1);
plot(t_hist, rad2deg(phi_true_h),'b','LineWidth',1.5); hold on;
plot(t_hist, rad2deg(phi_est_h),'r--','LineWidth',1.2);
ylabel('\phi [deg]'); title('Roll'); legend('True','Est'); grid on;
subplot(3,1,2);
plot(t_hist, rad2deg(theta_true_h),'b','LineWidth',1.5); hold on;
plot(t_hist, rad2deg(theta_est_h),'r--','LineWidth',1.2);
ylabel('\theta [deg]'); title('Pitch'); grid on;
subplot(3,1,3);
plot(t_hist, rad2deg(chi_true_h),'b','LineWidth',1.5); hold on;
plot(t_hist, rad2deg(chi_est_h),'r--','LineWidth',1.2);
ylabel('\chi [deg]'); title('Course/Heading'); xlabel('Time [s]'); grid on;

% --- Plot 4: Gyro bias convergence ---
figure('Name','Ch08: Gyro Bias','NumberTitle','off');
subplot(3,1,1);
plot(t_hist, rad2deg(bp_est_h),'b'); ylabel('b_p [deg/s]'); title('Gyro bias estimates'); grid on;
subplot(3,1,2);
plot(t_hist, rad2deg(bq_est_h),'r'); ylabel('b_q [deg/s]'); grid on;
subplot(3,1,3);
plot(t_hist, rad2deg(br_est_h),'g'); ylabel('b_r [deg/s]'); xlabel('Time [s]'); grid on;

% --- Plot 5: Innovations ---
figure('Name','Ch08: Innovations','NumberTitle','off');
subplot(4,1,1);
plot(t_hist, innov_baro_h,'b'); ylabel('\delta h [m]'); title('Baro innovation'); grid on;
subplot(4,1,2);
plot(t_hist, innov_pitot_h,'r'); ylabel('\delta Va [m/s]'); title('Pitot innovation'); grid on;
subplot(4,1,3);
plot(t_hist, rad2deg(innov_mag_h),'g'); ylabel('\delta\psi [deg]'); title('Mag innovation'); grid on;
subplot(4,1,4);
plot(t_hist, innov_gps_h(1,:),'b'); hold on;
plot(t_hist, innov_gps_h(2,:),'r');
ylabel('\delta GPS [m]'); xlabel('Time [s]'); title('GPS pn/pe innovation');
legend('pn','pe'); grid on;

fprintf('\n========================================\n');
fprintf('  Chapter 8 complete. 5 plot windows.\n');
fprintf('  Full pipeline: Sensors → EKF → Autopilot\n');
fprintf('========================================\n\n');
