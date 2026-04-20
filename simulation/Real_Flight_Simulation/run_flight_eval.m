% =========================================================================
%  RUN_FLIGHT_EVAL  —  Real Flight Data EKF Comparison
% =========================================================================
%  Reads raw sensor data from PX4 .ulg log file, runs AtlasFC EKF on
%  the same data, and compares with PX4 EKF2 output.
%
%  Question:
%    "Does our EKF produce better or worse estimates than PX4
%     with the same raw sensor data?"
%
%  Pipeline:
%    .ulg file
%      → load_ulg_sensors()   raw IMU / baro / GPS / mag
%      → load_ulg_ekf()       PX4 EKF2 estimates (reference)
%      → EKF loop             AtlasFC own EKF
%      → Comparison           RMSE, time series plots
%
%  Usage:
%    Run run_flight_eval.m with F5.
%    Log file: set path cfg.ulg_file in flight_cfg.m.
%
%  Author : AtlasFC
% =========================================================================

% -------------------------------------------------------------------------
%  0. Bootstrap — find atlas_root, setup paths
% -------------------------------------------------------------------------
candidate = pwd;
atlas_root = '';
for i = 1:10
    if exist(fullfile(candidate, 'setup_paths.m'), 'file')
        atlas_root = candidate;
        break;
    end
    parent = fileparts(candidate);
    if strcmp(parent, candidate), break; end
    candidate = parent;
end
if isempty(atlas_root)
    error('atlas_root not found. Run from inside the AtlasFC project folder.');
end
addpath(atlas_root);
setup_paths(atlas_root);

clc;
fprintf('=========================================\n');
fprintf('  Real Flight EKF Evaluation\n');
fprintf('=========================================\n\n');

% -------------------------------------------------------------------------
%  1. Configuration
% -------------------------------------------------------------------------
cfg = flight_cfg();

% Log file path: always pulled from log_files/ folder
script_dir   = fileparts(mfilename('fullpath'));
log_dir      = fullfile(script_dir, 'log_files');
if ~isabs_path(cfg.ulg_file)
    cfg.ulg_file = fullfile(log_dir, cfg.ulg_file);
end

if ~exist(cfg.ulg_file, 'file')
    error('Log file not found:\n  %s\nCheck cfg.ulg_file path in flight_cfg.m.', ...
          cfg.ulg_file);
end

fprintf('Log          : %s\n', cfg.ulg_file);
fprintf('dt           : %.3f s  (%.0f Hz)\n', cfg.dt, 1/cfg.dt);

% -------------------------------------------------------------------------
%  2. Open log
% -------------------------------------------------------------------------
fprintf('\nOpening log with ulogreader...\n');
ulog   = ulogreader(cfg.ulg_file);
t0_log = NaN;   % set in load_ulg_sensors

% -------------------------------------------------------------------------
%  3. Raw sensor data
% -------------------------------------------------------------------------
sensors = load_ulg_sensors(ulog, cfg);
N       = length(sensors.t);
dt      = cfg.dt;

% -------------------------------------------------------------------------
%  3b. Flight data statistics
% -------------------------------------------------------------------------
n_baro_upd  = sum(sensors.baro_valid);
n_gps_upd   = sum(sensors.gps_valid);
n_mag_upd   = sum(sensors.mag_valid);
imu_rate_hz = sensors.rate.imu;    % actual rate from log timestamps
baro_rate   = sensors.rate.baro;
gps_rate    = sensors.rate.gps;
mag_rate    = sensors.rate.mag;
h_min       = min(sensors.baro);
h_max       = max(sensors.baro);
Va_all      = sqrt(sum(sensors.gps(3,:).^2, 1));   % Va from GPS row
Va_mean     = mean(sensors.gps(3, sensors.gps_valid));
pN_range    = max(sensors.gps(1,:)) - min(sensors.gps(1,:));
pE_range    = max(sensors.gps(2,:)) - min(sensors.gps(2,:));

fprintf('\n');
fprintf('── Flight Data Summary ──────────────────────────────────────\n');
fprintf('  Log file         : %s\n',    cfg.ulg_file);
fprintf('  Flight duration  : %.1f s  (%.1f min)\n', sensors.t(end), sensors.t(end)/60);
fprintf('  Resampled steps  : %d  @ %.0f Hz  (dt = %.3f s)\n', N, imu_rate_hz, dt);
fprintf('  ── Sensor update rates ──\n');
fprintf('  IMU  (gyro/accel): %.0f Hz  (%d samples)\n', imu_rate_hz, N);
fprintf('  Barometer        : %.1f Hz  (%d updates)\n', baro_rate, n_baro_upd);
fprintf('  GPS              : %.1f Hz  (%d updates)\n', gps_rate,  n_gps_upd);
fprintf('  Magnetometer     : %.1f Hz  (%d updates)\n', mag_rate,  n_mag_upd);
fprintf('  ── Flight envelope ──\n');
fprintf('  Altitude         : %.1f m  →  %.1f m  (range %.1f m)\n', h_min, h_max, h_max-h_min);
fprintf('  Mean airspeed    : %.1f m/s\n', Va_mean);
fprintf('  Ground track     : %.0f m N  ×  %.0f m E\n', pN_range, pE_range);
fprintf('  GPS ref point    : lat=%.6f°  lon=%.6f°  alt=%.1f m\n', ...
        rad2deg(sensors.lat0), rad2deg(sensors.lon0), sensors.alt0);
fprintf('────────────────────────────────────────────────────────────\n');

% -------------------------------------------------------------------------
%  4. PX4 EKF2 reference data
% -------------------------------------------------------------------------
px4 = load_ulg_ekf(ulog, sensors.t, 0);   % t0_log=0 (already normalized)

% -------------------------------------------------------------------------
%  4b. Confirmation prompt  (Enter = start,  Esc = cancel)
% -------------------------------------------------------------------------
fprintf('\n');
key_pressed = confirm_prompt('Real Flight EKF Eval — Ready?', ...
    sprintf('ENTER  to run EKF on %.0f s of real flight data   |   ESC  to cancel', ...
            sensors.t(end)));

if ismember(key_pressed, {'escape','timeout'})
    if strcmp(key_pressed, 'timeout')
        fprintf('  No key pressed within 60 s — cancelled.\n');
    else
        fprintf('  Cancelled.\n');
    end
    return;
end
fprintf('  Starting...\n\n');

% -------------------------------------------------------------------------
%  5. AtlasFC EKF initialization
%     Take initial state from PX4 estimate (fair comparison: same start)
% -------------------------------------------------------------------------

% Select EKF version — adds core/estimator/ekf_vN/ to top of path
ekf_ver = 'ekf_v1';
if isfield(cfg, 'ekf_version'), ekf_ver = cfg.ekf_version; end
ekf_select(ekf_ver, atlas_root);
fprintf('  EKF version      : %s\n\n', ekf_ver);

params  = mav_params();
sparams = sensor_params();
ep      = ekf_params(sparams, cfg);   % cfg.R_* overrides hardware noise values

% Initial state: PX4's first estimate NED position+velocity + quaternion
phi0   = px4.phi(1);
theta0 = px4.theta(1);
psi0   = px4.psi(1);
q0     = euler_to_quaternion(phi0, theta0, psi0);

x0_13        = zeros(13,1);
x0_13(1)     = px4.pN(1);
x0_13(2)     = px4.pE(1);
x0_13(3)     = -px4.h(1);              % pd = -h
x0_13(4)     = px4.vN(1);
x0_13(5)     = px4.vE(1);
x0_13(6)     = px4.vD(1);
x0_13(7:10)  = q0;
x0_13(11:13) = zeros(3,1);             % start with zero bias

[x_hat, P] = ekf_init(x0_13, ep);

% Initial perturbation (if 0 in cfg, no effect)
x_hat(1:3) = x_hat(1:3) + cfg.ekf_pos_err * randn(3,1);
x_hat(4:6) = x_hat(4:6) + cfg.ekf_vel_err * randn(3,1);

% -------------------------------------------------------------------------
%  6. Pre-allocate output arrays
% -------------------------------------------------------------------------
pN_e    = zeros(1,N);  pE_e  = zeros(1,N);  h_e   = zeros(1,N);
vN_e    = zeros(1,N);  vE_e  = zeros(1,N);  vD_e  = zeros(1,N);
phi_e   = zeros(1,N);  theta_e = zeros(1,N); psi_e = zeros(1,N);
Va_e    = zeros(1,N);

% -------------------------------------------------------------------------
%  7. EKF loop  (atlas_sim's EKF block — no autopilot)
% -------------------------------------------------------------------------
BAR_W   = 36;
t_start = tic;
fprintf('Running AtlasFC EKF...\n');

for k = 1:N
    % Convert sensor packet to EKF format
    y.gyro      = sensors.gyro(:,k);
    y.accel     = sensors.accel(:,k);
    y.baro      = sensors.baro(k);
    y.baro_new  = sensors.baro_valid(k);
    y.pitot     = sensors.gps(3,k);    % Va_gps
    y.pitot_new = sensors.gps_valid(k);
    y.mag       = sensors.mag(k);
    y.mag_new   = sensors.mag_valid(k);
    y.gps       = sensors.gps(:,k);
    y.gps_new   = sensors.gps_valid(k);

    % EKF step
    [x_hat, P, ~] = ekf(x_hat, P, y, ep, params, dt);

    % Save estimates
    R_est         = quaternion_to_rotation(x_hat(7:10));
    [phi_est, theta_est, psi_est] = rotation_to_euler(R_est);

    pN_e(k)    = x_hat(1);
    pE_e(k)    = x_hat(2);
    h_e(k)     = -x_hat(3);
    vN_e(k)    = x_hat(4);
    vE_e(k)    = x_hat(5);
    vD_e(k)    = x_hat(6);
    phi_e(k)   = phi_est;
    theta_e(k) = theta_est;
    psi_e(k)   = psi_est;
    Va_e(k)    = sqrt(x_hat(4)^2 + x_hat(5)^2 + x_hat(6)^2);

    % --- Progress bar ---
    elapsed = toc(t_start);
    eta     = elapsed / k * (N - k);
    pct     = k / N;
    filled  = round(pct * BAR_W);
    bar_str = [repmat('#', 1, filled), repmat('-', 1, BAR_W - filled)];

    if k < N
        fprintf('\r  [%s] %3.0f%%  step %d/%d  elapsed %4.0fs  ETA %4.0fs  ', ...
                bar_str, pct*100, k, N, elapsed, eta);
    else
        fprintf('\r  [%s] 100%%  %d steps  total %.1f s               \n', ...
                bar_str, N, elapsed);
    end
end
fprintf('Done.\n\n');

% -------------------------------------------------------------------------
%  8. RMSE calculation  (after burn-in)
% -------------------------------------------------------------------------
k_burn = find(sensors.t >= cfg.rmse_burn_in, 1, 'first');
if isempty(k_burn), k_burn = 1; end

rmse = @(a, b) sqrt(mean((a(k_burn:end) - b(k_burn:end)).^2));
rad2deg_f = @(x) x * 180/pi;

fprintf('── RMSE Comparison  (t > %.0f s after) ──────────────\n', cfg.rmse_burn_in);
fprintf('  %-18s  AtlasFC      PX4-ref\n', 'Metric');
fprintf('  %-18s  %-12s %-12s\n', '------', '-----------', '-----------');

% AtlasFC vs PX4
rmse_pN    = rmse(pN_e,    px4.pN');
rmse_pE    = rmse(pE_e,    px4.pE');
rmse_h     = rmse(h_e,     px4.h');
rmse_phi   = rad2deg_f(rmse(phi_e,   px4.phi'));
rmse_theta = rad2deg_f(rmse(theta_e, px4.theta'));
rmse_psi   = rad2deg_f(rmse(psi_e,   px4.psi'));

fprintf('  %-18s  %.3f m\n',   'pN (north)',  rmse_pN);
fprintf('  %-18s  %.3f m\n',   'pE (east)',   rmse_pE);
fprintf('  %-18s  %.3f m\n',   'h  (altitude)', rmse_h);
fprintf('  %-18s  %.3f deg\n', 'phi  (roll)', rmse_phi);
fprintf('  %-18s  %.3f deg\n', 'theta (pitch)',rmse_theta);
fprintf('  %-18s  %.3f deg\n', 'psi  (yaw)',  rmse_psi);
fprintf('────────────────────────────────────────────────────────────\n');
fprintf('Note: PX4 EKF2 is used as reference in this analysis.\n\n');

% -------------------------------------------------------------------------
%  9. Plots
% -------------------------------------------------------------------------
t = sensors.t;

% --- Figure 1: Position comparison ---
figure('Name','AtlasFC vs PX4 — Position','NumberTitle','off');
subplot(3,1,1);
plot(t, pN_e, 'b-', t, px4.pN, 'r--', 'LineWidth',1.5);
ylabel('pN [m]'); title('North Position'); grid on; legend('AtlasFC','PX4');

subplot(3,1,2);
plot(t, pE_e, 'b-', t, px4.pE, 'r--', 'LineWidth',1.5);
ylabel('pE [m]'); title('East Position'); grid on;

subplot(3,1,3);
plot(t, h_e, 'b-', t, px4.h, 'r--', 'LineWidth',1.5);
ylabel('h [m]'); xlabel('Time [s]'); title('Altitude'); grid on;

% --- Figure 2: Attitude comparison ---
figure('Name','AtlasFC vs PX4 — Attitude','NumberTitle','off');
subplot(3,1,1);
plot(t, rad2deg(phi_e),   'b-', t, rad2deg(px4.phi'),   'r--', 'LineWidth',1.5);
ylabel('phi [deg]'); title('Roll'); grid on; legend('AtlasFC','PX4');

subplot(3,1,2);
plot(t, rad2deg(theta_e), 'b-', t, rad2deg(px4.theta'), 'r--', 'LineWidth',1.5);
ylabel('theta [deg]'); title('Pitch'); grid on;

subplot(3,1,3);
plot(t, rad2deg(psi_e),   'b-', t, rad2deg(px4.psi'),   'r--', 'LineWidth',1.5);
ylabel('psi [deg]'); xlabel('Time [s]'); title('Yaw'); grid on;

% --- Figure 3: Position error (AtlasFC - PX4) ---
figure('Name','AtlasFC — PX4 Difference','NumberTitle','off');
subplot(3,1,1);
plot(t, pN_e - px4.pN', 'b-', 'LineWidth',1.2); yline(0,'k--');
ylabel('\Delta pN [m]'); title('North Position Difference'); grid on;

subplot(3,1,2);
plot(t, pE_e - px4.pE', 'b-', 'LineWidth',1.2); yline(0,'k--');
ylabel('\Delta pE [m]'); title('East Position Difference'); grid on;

subplot(3,1,3);
plot(t, h_e - px4.h', 'b-', 'LineWidth',1.2); yline(0,'k--');
ylabel('\Delta h [m]'); xlabel('Time [s]'); title('Altitude Difference'); grid on;

% --- Figure 4: 2D trajectory ---
figure('Name','2D Trajectory — AtlasFC vs PX4','NumberTitle','off');
plot(pE_e, pN_e, 'b-', 'LineWidth',1.5); hold on;
plot(px4.pE, px4.pN, 'r--', 'LineWidth',1.5);
plot(pE_e(1), pN_e(1), 'go', 'MarkerSize',10, 'LineWidth',2);
hold off;
xlabel('pE [m] (East)'); ylabel('pN [m] (North)');
title('2D Trajectory'); legend('AtlasFC','PX4','Start'); grid on; axis equal;

fprintf('Plots completed.\n');

% =========================================================================
%  LOCAL HELPERS
% =========================================================================

function result = isabs_path(p)
% Checks whether path is absolute (Windows and Unix)
    result = ~isempty(p) && (p(1) == '/' || p(1) == '\' || ...
             (length(p) >= 2 && p(2) == ':'));
end

% -------------------------------------------------------------------------
function key = confirm_prompt(title_str, msg_str)
% Opens a small confirmation window.
% Returns 'return' (Enter), 'escape', or 'timeout' (no key within 60 s).
    hf = figure( ...
        'Name',        title_str, ...
        'NumberTitle', 'off', ...
        'MenuBar',     'none', ...
        'ToolBar',     'none', ...
        'Resize',      'off', ...
        'Position',    [500 450 460 70]);
    uicontrol(hf, ...
        'Style',               'text', ...
        'Units',               'normalized', ...
        'Position',            [0.02 0.15 0.96 0.70], ...
        'String',              msg_str, ...
        'FontSize',            11, ...
        'HorizontalAlignment', 'center', ...
        'BackgroundColor',     [0.94 0.94 0.94]);
    setappdata(hf, 'key', 'timeout');
    set(hf, 'KeyPressFcn', @(src, evt) cb_key(src, evt));
    figure(hf);
    uiwait(hf, 60);
    if isvalid(hf)
        key = getappdata(hf, 'key');
        close(hf);
    else
        key = 'timeout';
    end
end

% -------------------------------------------------------------------------
function cb_key(src, evt)
% KeyPressFcn: store key and resume uiwait on Enter or Esc only
    if ismember(evt.Key, {'return','escape'})
        setappdata(src, 'key', evt.Key);
        uiresume(src);
    end
end
