% =========================================================================
%  TEST_CH12_PLANNING - Unit tests for Ch12 RRT path planning
% =========================================================================
%  Tests the core path planning functions independently of atlas_sim.
%
%  Test groups:
%    Section 1 — collision_check : segment / cylinder intersection logic
%    Section 2 — smooth_path     : path shortcutting
%    Section 3 — rrt_plan        : full planner (open + cluttered map)
%
%  Run this file section-by-section (Ctrl+Enter) or as a whole.
%  Each test prints PASS / FAIL and a brief description.
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 12
% =========================================================================

clc;
fprintf('\n========================================\n');
fprintf('  AtlasFC  --  Ch12 Planning Unit Tests\n');
fprintf('========================================\n\n');

pass_total = 0;
fail_total = 0;

% =========================================================================
%% Section 1: collision_check tests
% =========================================================================
fprintf('--- Section 1: collision_check ---\n');

% Obstacle: cylinder at (500, 500), radius 100 m, height 200 m AGL
obs = [500, 500, 100, 200];

% Test 1.1 — Clear path: passes far from cylinder
p1 = [0; 0; -100];   p2 = [1000; 0; -100];
ok = collision_check(p1, p2, obs);
[pass_total, fail_total] = tally(ok == true, '1.1 Clear path (far from cylinder)', pass_total, fail_total);

% Test 1.2 — Head-on collision: segment passes through cylinder centre
p1 = [500; 0; -100];  p2 = [500; 1000; -100];
ok = collision_check(p1, p2, obs);
[pass_total, fail_total] = tally(ok == false, '1.2 Head-on through cylinder', pass_total, fail_total);

% Test 1.3 — Tangent path: segment just outside radius (1 m clearance)
p1 = [500-100-1; 0; -100];  p2 = [500-100-1; 1000; -100];
ok = collision_check(p1, p2, obs);
[pass_total, fail_total] = tally(ok == true, '1.3 Tangent 1 m outside cylinder', pass_total, fail_total);

% Test 1.4 — UAV above obstacle height: no collision even through cylinder
p1 = [500; 0; -250];  p2 = [500; 1000; -250];  % h=250 m > obstacle 200 m
ok = collision_check(p1, p2, obs);
[pass_total, fail_total] = tally(ok == true, '1.4 UAV above obstacle top (no collision)', pass_total, fail_total);

% Test 1.5 — Empty obstacle list: always clear
ok = collision_check([0;0;-100], [1000;1000;-100], []);
[pass_total, fail_total] = tally(ok == true, '1.5 Empty obstacle list', pass_total, fail_total);

% Test 1.6 — Multiple obstacles: diagonal hits first cylinder
obs_multi = [100, 100, 50, 200; 900, 900, 50, 200];
p1 = [0; 0; -100];  p2 = [1000; 1000; -100];
ok = collision_check(p1, p2, obs_multi);
[pass_total, fail_total] = tally(ok == false, '1.6 Diagonal blocked by multi-obstacle', pass_total, fail_total);

% Test 1.7 — Segment that misses all obstacles in multi-obstacle map
p1 = [0; 500; -100];  p2 = [1000; 500; -100];
ok = collision_check(p1, p2, obs_multi);
[pass_total, fail_total] = tally(ok == true, '1.7 Clear path in multi-obstacle map', pass_total, fail_total);

fprintf('\n');

% =========================================================================
%% Section 2: smooth_path tests
% =========================================================================
fprintf('--- Section 2: smooth_path ---\n');

% Test 2.1 — No obstacles: zigzag collapses to start + end
zigzag = [0,0,-100; 100,200,-100; 200,0,-100; 300,200,-100; 400,0,-100];
sm = smooth_path(zigzag, []);
[pass_total, fail_total] = tally(size(sm,1) <= 2, ...
    sprintf('2.1 No obstacles: zigzag → %d pts (expect ≤2)', size(sm,1)), pass_total, fail_total);

% Test 2.2 — Single waypoint returned unchanged
single = [0, 0, -100];
sm2 = smooth_path(single, []);
[pass_total, fail_total] = tally(size(sm2,1) == 1, '2.2 Single-point path unchanged', pass_total, fail_total);

% Test 2.3 — Two-point path unchanged
two = [0,0,-100; 500,500,-100];
sm3 = smooth_path(two, []);
[pass_total, fail_total] = tally(size(sm3,1) == 2, '2.3 Two-point path unchanged', pass_total, fail_total);

% Test 2.4 — Obstacle forces detour: path not fully collapsed
obs_block = [250, 250, 150, 200];
detour = [0,0,-100; 200,50,-100; 400,250,-100; 500,500,-100];
sm4 = smooth_path(detour, obs_block);
[pass_total, fail_total] = tally(size(sm4,1) >= 2, ...
    sprintf('2.4 Detour preserved: %d pts remain', size(sm4,1)), pass_total, fail_total);

fprintf('\n');

% =========================================================================
%% Section 3: rrt_plan tests
% =========================================================================
fprintf('--- Section 3: rrt_plan ---\n');

rng(0);   % Fixed seed

% --- 3a: Open map (no obstacles) ---
plan_open.start    = [0; 0; -100];
plan_open.goal     = [500; 500; -100];
plan_open.step     = 150;
plan_open.max_iter = 500;
plan_open.goal_rad = 150;
plan_open.p_goal   = 0.15;
plan_open.Va       = 25;

map_open.size_n    = 1000;
map_open.size_e    = 1000;
map_open.obstacles = [];

[wps, tree, ok_open] = rrt_plan(plan_open, map_open);

[pass_total, fail_total] = tally(ok_open, '3.1 RRT succeeds in open map', pass_total, fail_total);
[pass_total, fail_total] = tally(size(wps,2) == 4, '3.2 Waypoints are [N×4]', pass_total, fail_total);

if ok_open
    start_ok = norm(wps(1,1:3) - plan_open.start') < 1;
    goal_ok  = norm(wps(end,1:3) - plan_open.goal') < plan_open.goal_rad + 10;
    [pass_total, fail_total] = tally(start_ok && goal_ok, ...
        '3.3 Path starts at start, ends near goal', pass_total, fail_total);
end

if ok_open
    all_clear = true;
    for s = 1:size(wps,1)-1
        if ~collision_check(wps(s,1:3)', wps(s+1,1:3)', map_open.obstacles)
            all_clear = false; break;
        end
    end
    [pass_total, fail_total] = tally(all_clear, '3.4 All segments collision-free (open)', pass_total, fail_total);
end

[pass_total, fail_total] = tally(isfield(tree,'nodes') && isfield(tree,'parent'), ...
    '3.5 Tree struct has nodes + parent fields', pass_total, fail_total);

% --- 3b: Cluttered map (cfg_ch12) ---
rng(42);
cfg12  = cfg_ch12();
[wps2, tree2, ok2] = rrt_plan(cfg12.plan, cfg12.map);

[pass_total, fail_total] = tally(ok2, '3.6 RRT succeeds in cluttered map', pass_total, fail_total);

if ok2
    all_clear2 = true;
    for s = 1:size(wps2,1)-1
        if ~collision_check(wps2(s,1:3)', wps2(s+1,1:3)', cfg12.map.obstacles)
            all_clear2 = false; break;
        end
    end
    [pass_total, fail_total] = tally(all_clear2, '3.7 All segments collision-free (cluttered)', pass_total, fail_total);
end

[pass_total, fail_total] = tally(size(tree2.nodes,1) > 1, ...
    sprintf('3.8 Tree grew to %d nodes', size(tree2.nodes,1)), pass_total, fail_total);

if ok2
    direct_dist = norm(cfg12.plan.goal(1:2) - cfg12.plan.start(1:2));
    plan_pts    = wps2(:,1:2);
    plan_dist   = sum(sqrt(sum(diff(plan_pts,1,1).^2, 2)));
    [pass_total, fail_total] = tally(plan_dist < 4 * direct_dist, ...
        sprintf('3.9 Path length %.0f m < 4×direct (%.0f m)', plan_dist, direct_dist), ...
        pass_total, fail_total);
end

fprintf('\n');

% =========================================================================
%  Summary
% =========================================================================
total = pass_total + fail_total;
fprintf('========================================\n');
fprintf('  Results: %d / %d tests passed\n', pass_total, total);
if fail_total == 0
    fprintf('  All tests PASSED.\n');
else
    fprintf('  %d test(s) FAILED.\n', fail_total);
end
fprintf('========================================\n\n');

% =========================================================================
%  Local helpers
% =========================================================================

function [p, f] = tally(condition, label, p, f)
    if condition
        fprintf('  PASS  %s\n', label);
        p = p + 1;
    else
        fprintf('  FAIL  %s\n', label);
        f = f + 1;
    end
end
