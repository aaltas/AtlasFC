% =========================================================================
%  CH07_MAIN - Chapter 7: Sensor Models & Noise Characterization
% =========================================================================
%  Runs a 30-second open-loop trim simulation (no autopilot) and
%  records true states alongside all sensor measurements.  Plots
%  illustrate noise, bias drift, and GPS low-rate behavior.
%
%  Architecture context:
%    Ch5 → trim + transfer functions
%    Ch6 → PID autopilot (open-loop here, constant trim input)
%    Ch7 → sensor models (THIS FILE)
%    Ch8 → EKF state estimator using Ch7 sensor outputs
%
%  Simulation timeline:
%    t = 0..30 s : Straight-and-level flight at trim (Va=25, h=100m)
%    Open-loop: u_cmd = u_trim  (no autopilot active)
%
%  Test sections:
%    Section 1 - Sensor parameter summary
%    Section 2 - Open-loop simulation with sensor recording
%    Section 3 - Sensor noise statistics vs specification
%    Section 4 - Plots (IMU, air data, GPS)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 7
% =========================================================================

% --- Auto-path setup ---
atlas_root = fullfile(pwd, '..', '..');
addpath(atlas_root);
setup_paths(atlas_root);

clear; clc;
atlas_root = fullfile(pwd, '..', '..');
addpath(atlas_root);
setup_paths(atlas_root);

fprintf('\n========================================\n');
fprintf('  AtlasFC  --  Chapter 7: Sensor Models\n');
fprintf('========================================\n\n');

params  = mav_params();
sparams = sensor_params();

% =========================================================================
%  SECTION 1 — Sensor Parameter Summary
% =========================================================================
fprintf('--- SECTION 1: Sensor Specifications ---\n');
fprintf('  Gyro:      sigma=%.4f deg/s,  tau=%.0f s,  sigma_b=%.5f deg/s\n', ...
    rad2deg(sparams.sigma_gyro), sparams.tau_gyro, rad2deg(sparams.sigma_b_gyro));
fprintf('  Accel:     sigma=%.4f m/s^2, tau=%.0f s,  sigma_b=%.5f m/s^2\n', ...
    sparams.sigma_accel, sparams.tau_accel, sparams.sigma_b_accel);
fprintf('  Baro:      sigma=%.4f m,     tau=%.0f s\n', ...
    sparams.sigma_baro, sparams.tau_baro);
fprintf('  Pitot:     sigma=%.4f m/s,   tau=%.0f s\n', ...
    sparams.sigma_pitot, sparams.tau_pitot);
fprintf('  Mag:       sigma=%.3f deg\n', rad2deg(sparams.sigma_mag));
fprintf('  GPS pn/pe: sigma=%.3f m     (%.1f Hz update)\n', ...
    sparams.sigma_gps_pn, sparams.gps_hz);
fprintf('  GPS Vg:    sigma=%.3f m/s\n', sparams.sigma_gps_Vg);
fprintf('  GPS chi:   sigma=%.3f deg\n\n', rad2deg(sparams.sigma_gps_chi));

% =========================================================================
%  PREREQUISITE: Load trim data saved by Ch.5
% =========================================================================
params_dir = fileparts(which('mav_params'));
trim_file  = fullfile(params_dir, 'trim_data.mat');
if ~exist(trim_file, 'file')
    error('trim_data.mat not found. Run ch05_main.m first.');
end
load(trim_file, 'x_trim', 'u_trim', 'Va_star');

% Convert trim 12-state to 13-state (quaternion)
e_q_trim = euler_to_quaternion(x_trim(7), x_trim(8), x_trim(9));
x13_trim = [x_trim(1:6); e_q_trim; x_trim(10:12)];

% Override position to start at pn=0, pe=0, pd=-100m (h=100m)
x13_trim(1) = 0;
x13_trim(2) = 0;
x13_trim(3) = -100;

fprintf('Trim: Va*=%.1f m/s, h*=100 m, alpha*=%.2f deg\n\n', ...
    Va_star, rad2deg(x_trim(8)));

% =========================================================================
%  SECTION 2 — Open-Loop Simulation with Sensors
% =========================================================================
fprintf('--- SECTION 2: Open-Loop Simulation (30 s) ---\n');

dt   = 0.01;       % 100 Hz IMU
T    = 30.0;
N    = round(T / dt);

% --- Initialize sensor state ---
sensor_state = sensors_init(sparams, x13_trim);

% --- Pre-allocate ---
t_hist      = zeros(1, N);

% True states
Va_true_hist = zeros(1, N);
h_true_hist  = zeros(1, N);
psi_true_hist= zeros(1, N);
p_true_hist  = zeros(1, N);
q_true_hist  = zeros(1, N);
r_true_hist  = zeros(1, N);
ax_true_hist = zeros(1, N);
ay_true_hist = zeros(1, N);
az_true_hist = zeros(1, N);

% Sensor measurements
gyro_hist   = zeros(3, N);
accel_hist  = zeros(3, N);
baro_hist   = zeros(1, N);
pitot_hist  = zeros(1, N);
mag_hist    = zeros(1, N);
gps_pn_hist = zeros(1, N);
gps_pe_hist = zeros(1, N);
gps_Vg_hist = zeros(1, N);
gps_chi_hist= zeros(1, N);

% GPS flag: when does a new fix arrive?
gps_update_hist = false(1, N);

% --- Simulation state ---
x13 = x13_trim;

fprintf('  Running simulation...\n');
t0 = tic;

for k = 1:N
    t_now = (k-1) * dt;
    t_hist(k) = t_now;

    % --- Compute forces/moments at trim (open-loop) ---
    [fm, ~, ~, ~] = forces_moments(x13, u_trim, zeros(3,1), zeros(3,1), params);

    % --- True state observables ---
    Va_true_hist(k)  = sqrt(x13(4)^2 + x13(5)^2 + x13(6)^2);
    h_true_hist(k)   = -x13(3);
    R_bv = quaternion_to_rotation(x13(7:10));
    [~, ~, psi_k] = rotation_to_euler(R_bv);
    psi_true_hist(k) = psi_k;
    p_true_hist(k)   = x13(11);
    q_true_hist(k)   = x13(12);
    r_true_hist(k)   = x13(13);

    % True specific force (body) for comparison
    f_spec   = fm(1:3) / params.mass - R_bv * [0; 0; params.gravity];
    ax_true_hist(k) = f_spec(1);
    ay_true_hist(k) = f_spec(2);
    az_true_hist(k) = f_spec(3);

    % --- Sensor measurements ---
    gps_timer_before = sensor_state.gps_timer;
    [y, sensor_state] = sensors(x13, fm, sensor_state, sparams, params, dt);

    gyro_hist(:, k)    = y.gyro;
    accel_hist(:, k)   = y.accel;
    baro_hist(k)       = y.baro;
    pitot_hist(k)      = y.pitot;
    mag_hist(k)        = y.mag;
    gps_pn_hist(k)     = y.gps(1);
    gps_pe_hist(k)     = y.gps(2);
    gps_Vg_hist(k)     = y.gps(3);
    gps_chi_hist(k)    = y.gps(4);
    gps_update_hist(k) = (sensor_state.gps_timer > gps_timer_before - dt + 1e-9) && ...
                         (sensor_state.gps_timer > 0.95/sparams.gps_hz);

    % --- Propagate dynamics ---
    x13 = mav_dynamics(x13, fm, params, dt);
end

sim_time = toc(t0);
fprintf('  Done in %.2f s  (%d steps, %d GPS updates)\n', ...
    sim_time, N, sum(gps_update_hist));

% =========================================================================
%  SECTION 3 — Sensor Noise Statistics
% =========================================================================
fprintf('\n--- SECTION 3: Noise Statistics ---\n');

% Gyro
gyro_err = gyro_hist - [p_true_hist; q_true_hist; r_true_hist];
fprintf('\n  GYRO errors (std vs spec):\n');
for ax = 1:3
    ax_name = 'pqr';
    fprintf('    %c-axis: std=%.5f rad/s  (spec %.4f rad/s)\n', ...
        ax_name(ax), std(gyro_err(ax,:)), sparams.sigma_gyro);
end

% Accel
accel_err = accel_hist - [ax_true_hist; ay_true_hist; az_true_hist];
fprintf('\n  ACCEL errors (std vs spec):\n');
for ax = 1:3
    ax_name = 'xyz';
    fprintf('    %c-axis: std=%.5f m/s^2  (spec %.4f m/s^2)\n', ...
        ax_name(ax), std(accel_err(ax,:)), sparams.sigma_accel);
end

% Baro
baro_err = baro_hist - h_true_hist;
fprintf('\n  BARO error: std=%.4f m  (spec %.4f m)\n', ...
    std(baro_err), sparams.sigma_baro);

% Pitot
pitot_err = pitot_hist - Va_true_hist;
fprintf('  PITOT error: std=%.5f m/s  (spec %.4f m/s)\n', ...
    std(pitot_err), sparams.sigma_pitot);

% Mag
mag_err = mag_hist - psi_true_hist;
mag_err = atan2(sin(mag_err), cos(mag_err));  % wrap
fprintf('  MAG error:   std=%.5f rad  (spec %.4f rad)\n', ...
    std(mag_err), sparams.sigma_mag);

% GPS (only at update instants)
gps_idx = gps_update_hist;
if sum(gps_idx) > 2
    gps_pn_err = gps_pn_hist(gps_idx) - x13_trim(1);  % trim pn=0
    gps_pe_err = gps_pe_hist(gps_idx) - x13_trim(2);  % trim pe=0
    fprintf('  GPS pn err:  std=%.4f m   (spec %.4f m)\n', ...
        std(gps_pn_err), sparams.sigma_gps_pn);
    fprintf('  GPS pe err:  std=%.4f m   (spec %.4f m)\n', ...
        std(gps_pe_err), sparams.sigma_gps_pe);
end

% =========================================================================
%  SECTION 4 — Plots
% =========================================================================

% --- Plot 1: Rate Gyroscope ---
figure('Name', 'Ch07: Rate Gyroscope', 'NumberTitle', 'off');
labels = {'p [rad/s]', 'q [rad/s]', 'r [rad/s]'};
true_data = {p_true_hist, q_true_hist, r_true_hist};
for i = 1:3
    subplot(3,1,i);
    plot(t_hist, gyro_hist(i,:), 'b', 'LineWidth', 0.8); hold on;
    plot(t_hist, true_data{i}, 'r--', 'LineWidth', 1.2);
    ylabel(labels{i}); grid on;
    if i == 1; title('Rate Gyroscope: measured vs true'); end
    if i == 3; xlabel('Time [s]'); end
    if i == 1; legend('Measured', 'True'); end
end

% --- Plot 2: Accelerometer ---
figure('Name', 'Ch07: Accelerometer', 'NumberTitle', 'off');
labels_a = {'fx [m/s^2]', 'fy [m/s^2]', 'fz [m/s^2]'};
true_acc  = {ax_true_hist, ay_true_hist, az_true_hist};
for i = 1:3
    subplot(3,1,i);
    plot(t_hist, accel_hist(i,:), 'b', 'LineWidth', 0.8); hold on;
    plot(t_hist, true_acc{i}, 'r--', 'LineWidth', 1.2);
    ylabel(labels_a{i}); grid on;
    if i == 1; title('Accelerometer: measured vs true specific force'); end
    if i == 3; xlabel('Time [s]'); end
    if i == 1; legend('Measured', 'True'); end
end

% --- Plot 3: Air Data (Baro + Pitot) ---
figure('Name', 'Ch07: Air Data Sensors', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t_hist, baro_hist, 'b', 'LineWidth', 0.8); hold on;
plot(t_hist, h_true_hist, 'r--', 'LineWidth', 1.2);
ylabel('h [m]'); grid on;
title('Barometric Altimeter'); legend('Baro (measured)', 'True altitude');

subplot(2,1,2);
plot(t_hist, pitot_hist, 'b', 'LineWidth', 0.8); hold on;
plot(t_hist, Va_true_hist, 'r--', 'LineWidth', 1.2);
ylabel('Va [m/s]'); xlabel('Time [s]'); grid on;
title('Pitot Tube Airspeed'); legend('Pitot (measured)', 'True Va');

% --- Plot 4: Magnetometer ---
figure('Name', 'Ch07: Magnetometer', 'NumberTitle', 'off');
plot(t_hist, rad2deg(mag_hist), 'b', 'LineWidth', 0.8); hold on;
plot(t_hist, rad2deg(psi_true_hist), 'r--', 'LineWidth', 1.2);
ylabel('\psi [deg]'); xlabel('Time [s]'); grid on;
title('Magnetometer: measured heading vs true');
legend('Mag (measured)', 'True \psi');

% --- Plot 5: GPS (step plot — shows 1 Hz holds) ---
figure('Name', 'Ch07: GPS', 'NumberTitle', 'off');
subplot(2,2,1);
stairs(t_hist, gps_pn_hist, 'b', 'LineWidth', 1.0); hold on;
plot(t_hist, zeros(1,N), 'r--', 'LineWidth', 1.0);
ylabel('pn [m]'); title('GPS North'); grid on;

subplot(2,2,2);
stairs(t_hist, gps_pe_hist, 'b', 'LineWidth', 1.0); hold on;
plot(t_hist, zeros(1,N), 'r--', 'LineWidth', 1.0);
ylabel('pe [m]'); title('GPS East'); grid on;

subplot(2,2,3);
stairs(t_hist, gps_Vg_hist, 'b', 'LineWidth', 1.0); hold on;
plot(t_hist, Va_true_hist, 'r--', 'LineWidth', 1.0);
ylabel('Vg [m/s]'); xlabel('Time [s]'); title('GPS Ground Speed'); grid on;

subplot(2,2,4);
stairs(t_hist, rad2deg(gps_chi_hist), 'b', 'LineWidth', 1.0); hold on;
plot(t_hist, rad2deg(psi_true_hist), 'r--', 'LineWidth', 1.0);
ylabel('\chi [deg]'); xlabel('Time [s]'); title('GPS Course'); grid on;

fprintf('\n========================================\n');
fprintf('  Chapter 7 complete. 5 plot windows.\n');
fprintf('========================================\n\n');
fprintf('  Sensor outputs ready for Ch8 EKF input.\n');
fprintf('  Next: Ch8 will estimate [Va, h, phi, theta, psi]\n');
fprintf('        from y.gyro, y.accel, y.baro, y.pitot, y.mag, y.gps\n\n');
