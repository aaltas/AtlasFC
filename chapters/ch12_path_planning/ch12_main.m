% =========================================================================
%  CH12_MAIN - Chapter 12: RRT Path Planning
% =========================================================================
%  Demonstrates online path planning using the Rapidly-exploring Random
%  Tree (RRT) algorithm in a 2D obstacle field.
%
%  Pipeline:
%    1. Define map with cylinder obstacles
%    2. Run RRT planner → collision-free waypoint list
%    3. Smooth path (greedy shortcutting)
%    4. Fly planned path with Ch9 straight-line guidance (atlas_sim)
%    5. Analyse: obstacle clearance, path length, tracking performance
%
%  Sections:
%    Section 1 — Map and planning setup
%    Section 2 — RRT planning
%    Section 3 — Closed-loop simulation (Ch9 guidance on planned path)
%    Section 4 — Performance analysis
%    Section 5 — Plots (map + RRT tree, planned vs flown, altitude, course)
%
%  Run from the AtlasFC project root after setup_paths.
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 12
% =========================================================================

% --- Auto-path setup ---
current_file = mfilename('fullpath');
[script_dir, ~, ~] = fileparts(current_file);
atlas_root = fullfile(script_dir, '..', '..');

addpath(atlas_root);
if ~exist('setup_paths', 'file')
    error('setup_paths.m not found. Check atlas_root path.');
end
setup_paths(atlas_root);

clc; rng(42);   % Fixed seed for reproducible RRT

fprintf('\n========================================\n');
fprintf('  AtlasFC  --  Chapter 12: RRT Path Planning\n');
fprintf('========================================\n\n');

% =========================================================================
%  SECTION 1 — Map and planning setup
% =========================================================================
fprintf('--- SECTION 1: Map and Planning Setup ---\n');

cfg = cfg_ch12();

fprintf('  Map:  %d × %d m\n', cfg.map.size_n, cfg.map.size_e);
fprintf('  Start: [%.0f, %.0f] m\n', cfg.plan.start(1), cfg.plan.start(2));
fprintf('  Goal:  [%.0f, %.0f] m  (direct dist = %.0f m)\n', ...
    cfg.plan.goal(1), cfg.plan.goal(2), ...
    norm(cfg.plan.goal(1:2) - cfg.plan.start(1:2)));
fprintf('  Obstacles: %d cylinders\n', size(cfg.map.obstacles, 1));
fprintf('  RRT step = %.0f m,  max_iter = %d,  p_goal = %.2f\n\n', ...
    cfg.plan.step, cfg.plan.max_iter, cfg.plan.p_goal);

% Print obstacle table
fprintf('  %-8s %-10s %-10s %-10s %-10s\n', 'ID', 'North[m]', 'East[m]', 'Radius[m]', 'Height[m]');
for i = 1:size(cfg.map.obstacles, 1)
    obs = cfg.map.obstacles(i,:);
    fprintf('  OBS-%-4d %-10.0f %-10.0f %-10.0f %-10.0f\n', i, obs(1), obs(2), obs(3), obs(4));
end
fprintf('\n');

% =========================================================================
%  SECTION 2 — RRT path planning
% =========================================================================
fprintf('--- SECTION 2: RRT Planning ---\n');

t_rrt = tic;
[planned_wps, tree, rrt_ok] = rrt_plan(cfg.plan, cfg.map);
rrt_time = toc(t_rrt);

if rrt_ok
    fprintf('  RRT SUCCESS  (%.3f s)\n', rrt_time);
else
    fprintf('  RRT FAILED — using fallback direct path\n');
end

N_wp     = size(planned_wps, 1);
raw_dist = path_length_2d(planned_wps(:,1:2));
fprintf('  Planned waypoints (after smoothing): %d\n', N_wp);
fprintf('  Planned path length: %.0f m\n', raw_dist);
fprintf('  Tree nodes explored: %d\n\n', size(tree.nodes, 1));

% Print waypoint list
fprintf('  %-6s %-10s %-10s %-10s\n', 'WP#', 'North[m]', 'East[m]', 'Va[m/s]');
for i = 1:N_wp
    fprintf('  WP%-4d %-10.0f %-10.0f %-10.1f\n', i, ...
        planned_wps(i,1), planned_wps(i,2), planned_wps(i,4));
end
fprintf('\n');

% Verify obstacle clearance on planned path
fprintf('  Obstacle clearance check (planned path segments):\n');
all_clear = true;
for s = 1:N_wp-1
    p1 = planned_wps(s,   1:3)';
    p2 = planned_wps(s+1, 1:3)';
    ok = collision_check(p1, p2, cfg.map.obstacles);
    if ~ok
        fprintf('    Segment %d→%d: COLLISION DETECTED!\n', s, s+1);
        all_clear = false;
    end
end
if all_clear
    fprintf('    All segments clear.\n');
end
fprintf('\n');

% =========================================================================
%  SECTION 3 — Closed-loop simulation
% =========================================================================
fprintf('--- SECTION 3: Closed-Loop Simulation (%.0f s) ---\n', cfg.T);

% Inject planned waypoints into config
cfg.waypoints = planned_wps;

fprintf('  Running atlas_sim with RRT-planned waypoints...\n');
t_sim = tic;
results = atlas_sim(cfg);
sim_elapsed = toc(t_sim);
fprintf('  Done in %.2f s (%d steps).\n\n', sim_elapsed, length(results.t));

% =========================================================================
%  SECTION 4 — Performance analysis
% =========================================================================
fprintf('--- SECTION 4: Performance Analysis ---\n');

t    = results.t;
pn_t = results.true.pn;
pe_t = results.true.pe;
h_t  = results.true.h;
Va_t = results.true.Va;
chi_t = results.true.chi;

N = length(t);

% --- Path length flown ---
flown_dist = sum(sqrt(diff(pn_t).^2 + diff(pe_t).^2));
fprintf('  Planned path length: %.0f m\n', raw_dist);
fprintf('  Flown  path length:  %.0f m\n', flown_dist);

% --- Goal proximity ---
goal_n = cfg.plan.goal(1); goal_e = cfg.plan.goal(2);
dists_to_goal = sqrt((pn_t - goal_n).^2 + (pe_t - goal_e).^2);
[min_goal_dist, idx_closest] = min(dists_to_goal);
t_closest     = t(idx_closest);
final_pn = pn_t(end);  final_pe = pe_t(end);
fprintf('  Final position:         (%.0f, %.0f) m\n', final_pn, final_pe);
fprintf('  Closest approach to goal: %.1f m  at t=%.1f s\n', min_goal_dist, t_closest);
fprintf('  (Sim end distance:        %.1f m — UAV overflew after goal)\n', dists_to_goal(end));

% --- Altitude tracking ---
h_nom  = 100.0;
h_err  = h_t - h_nom;
idx_ss = max(1, round(N/4)) : N;
fprintf('\n  Altitude from 100 m nominal:\n');
fprintf('    RMS: %.2f m   Max: %.2f m\n', rms_val(h_err(idx_ss)), max(abs(h_err(idx_ss))));

% --- Airspeed tracking ---
Va_err = Va_t - cfg.plan.Va;
fprintf('  Airspeed from %.0f m/s:\n', cfg.plan.Va);
fprintf('    RMS: %.3f m/s  Max: %.3f m/s\n', rms_val(Va_err(idx_ss)), max(abs(Va_err(idx_ss))));

% --- Minimum obstacle clearance (flown trajectory) ---
fprintf('\n  Obstacle clearance (flown trajectory):\n');
min_clearance = inf;
for i = 1:size(cfg.map.obstacles,1)
    cn = cfg.map.obstacles(i,1); ce = cfg.map.obstacles(i,2);
    r  = cfg.map.obstacles(i,3);
    dists = sqrt((pn_t - cn).^2 + (pe_t - ce).^2);
    clearance = min(dists) - r;
    fprintf('    OBS-%d: min clearance = %.1f m\n', i, clearance);
    min_clearance = min(min_clearance, clearance);
end
if min_clearance > 0
    fprintf('  All obstacles cleared. Minimum clearance: %.1f m\n', min_clearance);
else
    fprintf('  WARNING: Trajectory entered obstacle zone (min clearance = %.1f m)\n', min_clearance);
end
fprintf('\n');

% =========================================================================
%  SECTION 5 — Plots
% =========================================================================
fprintf('--- SECTION 5: Plots ---\n');

OBS = cfg.map.obstacles;

% -- Figure 1: Map overview — RRT tree + planned path + flown trajectory --
figure(1); clf;
hold on;

% Draw RRT tree (grey edges)
for i = 2:size(tree.nodes, 1)
    pidx = tree.parent(i);
    if pidx > 0
        plot([tree.nodes(pidx,2), tree.nodes(i,2)], ...
             [tree.nodes(pidx,1), tree.nodes(i,1)], ...
             '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.5);
    end
end

% Draw obstacles (circles)
theta_c = linspace(0, 2*pi, 60);
for i = 1:size(OBS,1)
    cx = OBS(i,2); cy = OBS(i,1); r = OBS(i,3);
    fill(cx + r*cos(theta_c), cy + r*sin(theta_c), ...
         [0.85 0.4 0.4], 'FaceAlpha', 0.5, 'EdgeColor', [0.6 0 0]);
    text(cx, cy, sprintf('OBS%d', i), 'HorizontalAlignment','center', ...
         'FontSize', 7, 'Color', [0.5 0 0]);
end

% Planned path
plot(planned_wps(:,2), planned_wps(:,1), 'g-o', 'LineWidth', 2.5, ...
     'MarkerSize', 7, 'MarkerFaceColor', 'g', 'DisplayName', 'Planned path');

% Flown trajectory
plot(pe_t, pn_t, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Flown trajectory');

% Start and goal markers
plot(cfg.plan.start(2), cfg.plan.start(1), 'bs', 'MarkerSize', 12, ...
     'MarkerFaceColor', 'b', 'DisplayName', 'Start');
plot(cfg.plan.goal(2),  cfg.plan.goal(1),  'r*', 'MarkerSize', 14, ...
     'LineWidth', 2, 'DisplayName', 'Goal');

% Waypoint labels
for i = 1:N_wp
    text(planned_wps(i,2)+25, planned_wps(i,1)+25, sprintf('WP%d',i), ...
         'FontSize', 8, 'Color', [0 0.5 0]);
end

xlabel('East [m]'); ylabel('North [m]');
title('Ch12: RRT Path Planning — Map, Tree, and Flown Trajectory');
legend('Location','northwest'); grid on; axis equal;
xlim([-100, cfg.map.size_e+100]); ylim([-100, cfg.map.size_n+100]);

% -- Figure 2: Altitude vs time --
figure(2); clf;
plot(t, h_t, 'b-', 'LineWidth', 1.5);
hold on;
yline(100, 'k--', 'Nominal 100 m', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Altitude [m]');
title('Ch12: Altitude Tracking');
legend({'True altitude', 'Nominal 100 m'}); grid on;

% -- Figure 3: Course angle vs time --
figure(3); clf;
plot(t, rad2deg(chi_t), 'b-', 'LineWidth', 1.5);
hold on;
plot(t, rad2deg(results.cmd.chi), 'r--', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Course [deg]');
title('Ch12: Course Angle — True vs Commanded');
legend({'True', 'Commanded'}); grid on;

% -- Figure 4: Airspeed vs time --
figure(4); clf;
plot(t, Va_t, 'b-', 'LineWidth', 1.5);
hold on;
yline(cfg.plan.Va, 'r--', sprintf('Cmd %.0f m/s', cfg.plan.Va), 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Airspeed [m/s]');
title('Ch12: Airspeed Tracking');
legend({'True Va', 'Commanded'}); grid on;

% -- Figure 5: Obstacle clearance vs time --
figure(5); clf;
clr = zeros(N, size(OBS,1));
for i = 1:size(OBS,1)
    cn = OBS(i,1); ce = OBS(i,2); r = OBS(i,3);
    clr(:,i) = sqrt((pn_t - cn).^2 + (pe_t - ce).^2)' - r;
end
min_clr = min(clr, [], 2);
plot(t, min_clr, 'b-', 'LineWidth', 1.5);
hold on;
yline(0, 'r--', 'Obstacle surface', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Clearance [m]');
title('Ch12: Minimum Obstacle Clearance vs Time');
grid on;

fprintf('\n========================================\n');
if rrt_ok
    fprintf('  Chapter 12 complete — RRT succeeded.\n');
else
    fprintf('  Chapter 12 complete — RRT fallback used.\n');
end
fprintf('  Planned %d waypoints, %.0f m path.\n', N_wp, raw_dist);
fprintf('  5 plot windows generated.\n');
fprintf('========================================\n\n');

% =========================================================================
%  Local helpers
% =========================================================================

function L = path_length_2d(pts)
    d = diff(pts, 1, 1);
    L = sum(sqrt(sum(d.^2, 2)));
end

function y = rms_val(x)
    y = sqrt(mean(x.^2));
end
