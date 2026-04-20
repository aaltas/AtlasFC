% =========================================================================
%  CH09_MAIN - Chapter 9: Straight-Line and Circular Path Following
% =========================================================================
%  Implements and demonstrates the path following guidance laws from
%  Beard & McLain Chapter 9.
%
%  Runs a 200-second closed-loop simulation with waypoint-based guidance:
%    Waypoints form a rectangular loop at 100m altitude
%    Vehicle follows straight-line segments via lateral guidance law
%    Half-plane switching controls waypoint transitions
%
%  Sections:
%    Section 1 — Guidance parameter summary
%    Section 2 — Closed-loop simulation (200 s)
%    Section 3 — Path-following performance analysis
%    Section 4 — Plots (trajectory, altitude, course, cross-track error)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 9
% =========================================================================

% --- Auto-path setup ---
% Use upward search to find setup_paths
current_file = mfilename('fullpath');
[script_dir, ~, ~] = fileparts(current_file);
atlas_root = fullfile(script_dir, '..', '..');

% Add atlas root and run setup_paths
addpath(atlas_root);
if ~exist('setup_paths', 'file')
    error('setup_paths.m not found. Check atlas_root path.');
end
setup_paths(atlas_root);

% Clear and initialize
clear; clc; rng(0);
fprintf('\n========================================\n');
fprintf('  AtlasFC  --  Chapter 9: Path Following\n');
fprintf('========================================\n\n');

% =========================================================================
%  SECTION 1 — Guidance parameter summary
% =========================================================================
fprintf('--- Guidance Parameters ---\n');
fprintf('  Straight-line following:\n');
fprintf('    chi_inf = π/4 rad (45°)  — max approach angle\n');
fprintf('    k_path  = 0.025          — cross-track error gain\n');
fprintf('  Orbit following:\n');
fprintf('    k_orbit = 4.0            — radial error gain\n\n');

% Load MAV parameters
params  = mav_params();
sparams = sensor_params();
ep      = ekf_params(sparams);

% =========================================================================
%  Load trim data (prerequisite from Ch5)
% =========================================================================
params_dir = fileparts(which('mav_params'));
trim_file  = fullfile(params_dir, 'trim_data.mat');
if ~exist(trim_file, 'file')
    error('trim_data.mat not found. Run ch05_main.m first.');
end
load(trim_file, 'x_trim', 'u_trim', 'gains', 'Va_star');

% Build 13-state trim
e_q_trim = euler_to_quaternion(x_trim(7), x_trim(8), x_trim(9));
x13_trim = [x_trim(1:6); e_q_trim; x_trim(10:12)];
x13_trim(1:3) = [0; 0; -100];   % Start at h=100m

fprintf('Trim: Va*=%.1f m/s, h*=100m, alpha*=%.2f deg\n\n', ...
    Va_star, rad2deg(x_trim(8)));

% =========================================================================
%  SECTION 2 — Run simulation via atlas_sim
% =========================================================================
fprintf('--- SECTION 2: Path-Following Simulation (200 s) ---\n');

% Build config
cfg = cfg_ch09();

fprintf('  Waypoints: [%d x 4] matrix\n', size(cfg.waypoints, 1));
fprintf('    Rectangular pattern at 100m altitude\n');
fprintf('  Simulation time: %.1f s (%.0f Hz, %d steps)\n\n', ...
    cfg.T, 1/cfg.dt, round(cfg.T/cfg.dt));

fprintf('  Running simulation...\n');
t0_sim = tic;
results = atlas_sim(cfg);
sim_time = toc(t0_sim);

fprintf('  Done in %.2f s (%d steps).\n', sim_time, length(results.t));

% =========================================================================
%  SECTION 3 — Path-following performance
% =========================================================================
fprintf('\n--- SECTION 3: Path-Following Performance ---\n');

% Extract states
t = results.t;
pn_true = results.true.pn;
pe_true = results.true.pe;
h_true = results.true.h;
chi_true = results.true.chi;
pn_est = results.est.pn;
pe_est = results.est.pe;
h_est = results.est.h;
chi_est = results.est.chi;

% Analyze waypoint transitions
waypoints = cfg.waypoints;
N_wp = size(waypoints, 1);
fprintf('  Waypoint transitions:\n');
for i = 1:(N_wp-1)
    wp_curr = waypoints(i, 1:3);

    % Distance to this waypoint along trajectory
    dist_to_wp = sqrt((pn_true - wp_curr(1)).^2 + ...
                      (pe_true - wp_curr(2)).^2 + ...
                      (h_true  - wp_curr(3)).^2);
    [min_dist, idx] = min(dist_to_wp);

    fprintf('    WP%d: t=%.1f s, min_dist=%.2f m\n', i, t(idx), min_dist);
end

% Summary statistics
fprintf('\n  Altitude tracking:\n');
h_err = h_est - h_true;
fprintf('    RMS h error:   %.2f m\n', rms(h_err));
fprintf('    Max h error:   %.2f m\n', max(abs(h_err)));

fprintf('\n  Course tracking:\n');
chi_err = atan2(sin(chi_est - chi_true), cos(chi_est - chi_true));
fprintf('    RMS chi error: %.2f deg\n', rad2deg(rms(chi_err)));
fprintf('    Max chi error: %.2f deg\n', rad2deg(max(abs(chi_err))));

fprintf('\n  Airspeed tracking:\n');
va_err = results.est.Va - results.true.Va;
fprintf('    RMS Va error:  %.2f m/s\n', rms(va_err));
fprintf('    Max Va error:  %.2f m/s\n', max(abs(va_err)));

% =========================================================================
%  SECTION 4 — Plots
% =========================================================================

% Plot 0: 2D Trajectory with waypoints
fprintf('  Plotting path following results...\n');
figure('Name','Ch09: 2D Trajectory','NumberTitle','off');
plot(pe_true, pn_true, 'b-', 'LineWidth', 1.5); hold on;
plot(waypoints(:,2), waypoints(:,1), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
for i = 1:size(waypoints,1)
    text(waypoints(i,2)+10, waypoints(i,1)+10, sprintf('WP%d', i), 'FontSize', 10);
end
xlabel('East [m]');
ylabel('North [m]');
title('2D Flight Path with Waypoints');
grid on; axis equal;
legend('True trajectory', 'Waypoints');

% Plot 2: Altitude vs time
figure('Name','Ch09: Altitude','NumberTitle','off');
plot(t, h_true, 'b', 'LineWidth', 1.5); hold on;
plot(t, results.cmd.h, 'r--', 'LineWidth', 1.2);
ylabel('h [m]');
title('Altitude: True vs Commanded');
legend('True', 'Commanded');
grid on;

% Plot 3: Course angle vs time
figure('Name','Ch09: Course','NumberTitle','off');
plot(t, rad2deg(chi_true), 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(results.cmd.chi), 'r--', 'LineWidth', 1.2);
ylabel('\chi [deg]');
xlabel('Time [s]');
title('Course Heading: True vs Commanded');
legend('True', 'Commanded');
grid on;

% Plot 4: Airspeed
figure('Name','Ch09: Airspeed','NumberTitle','off');
plot(t, results.true.Va, 'b', 'LineWidth', 1.5); hold on;
plot(t, results.cmd.Va, 'r--', 'LineWidth', 1.2);
ylabel('Va [m/s]');
xlabel('Time [s]');
title('Airspeed: True vs Commanded');
legend('True', 'Commanded');
grid on;

fprintf('\n========================================\n');
fprintf('  Chapter 9 complete. 5 plot windows.\n');
fprintf('  Path following with guidance laws.\n');
fprintf('========================================\n\n');
