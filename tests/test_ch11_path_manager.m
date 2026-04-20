% =========================================================================
%  TEST_CH11_PATH_MANAGER - Unit tests for Ch11 path managers
% =========================================================================
%  Tests the fillet path manager geometry and switching logic.
%  Also smoke-tests the Dubins switching fix.
%
%  Test groups:
%    Section 1 — Fillet geometry (z1, z2, orbit center, lambda)
%    Section 2 — Fillet state machine (line→orbit→next leg transitions)
%    Section 3 — Dubins manager smoke test (no stuck-in-circle regression)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 11
% =========================================================================

clc;
fprintf('\n========================================\n');
fprintf('  AtlasFC  --  Ch11 Path Manager Tests\n');
fprintf('========================================\n\n');

pass_total = 0;
fail_total = 0;

% =========================================================================
%% Section 1: Fillet geometry
% =========================================================================
fprintf('--- Section 1: Fillet geometry ---\n');

% Waypoints: simple L-shaped path (north then east)
%   WP1=(0,0)  WP2=(400,0)  WP3=(400,400)
WP3 = [0,0,-100,25; 400,0,-100,25; 400,400,-100,25];
R = 80;
cfg_f.R_fillet       = R;
cfg_f.loop_waypoints = false;
cfg_f.use_fillet     = true;

% Manual geometry for turn at WP2:
%   q_i   = [1,0] (north, pn direction)
%   q_n   = [0,1] (east,  pe direction)
%   varrho = arccos(-[1,0]·[0,1]) = arccos(0) = pi/2 = 90°
%   d_f   = R/tan(pi/4) = R/1 = R = 80 m
%   z1    = WP2 - d*q_i = [400-80, 0] = [320, 0]
%   z2    = WP2 + d*q_n = [400, 80]
%   lambda= sign(q_i(n)*q_n(e) - q_i(e)*q_n(n)) = sign(1*1-0*0) = +1 (CW)
%   rp2   = [-q_i(2); q_i(1)] = [0; 1]  (east perp in pn,pe)
%   c     = z1 + R*lambda*rp2 = [320,0] + 80*[0;1] = [320, 80]

z1_expected = [320; 0; -100];
z2_expected = [400; 80; -100];
c_expected  = [320; 80; -100];  % [pn; pe; pd]
lambda_expected = 1;

% Test 1.1 — Fillet starts in straight-line phase
state1.wp_idx    = 1;
state1.fillet_seg = 1;
pos_before = [100; 0; -100];  % On leg 1, far from WP2
[path1, ~] = path_manager_fillet(WP3, pos_before, state1, cfg_f);
[pass_total, fail_total] = tally(strcmp(path1.type,'line'), ...
    '1.1 Initial state: straight-line path', pass_total, fail_total);

% Test 1.2 — Orbit starts when UAV crosses z1 half-plane
%   z1 is at pn=320; cross when pn >= 320 in q_i=[1,0] direction
state2.wp_idx    = 1;
state2.fillet_seg = 1;
pos_at_z1 = [321; 0; -100];   % Just past z1
[path2, state2] = path_manager_fillet(WP3, pos_at_z1, state2, cfg_f);
[pass_total, fail_total] = tally(strcmp(path2.type,'orbit'), ...
    '1.2 Past z1: orbit phase engaged', pass_total, fail_total);

% Test 1.3 — Orbit center matches expected [320, 80]
if strcmp(path2.type,'orbit')
    c_err = norm(path2.c(1:2) - c_expected(1:2));
    [pass_total, fail_total] = tally(c_err < 2, ...
        sprintf('1.3 Orbit center correct (err=%.1f m)', c_err), pass_total, fail_total);
end

% Test 1.4 — Orbit radius = R_fillet
if strcmp(path2.type,'orbit')
    [pass_total, fail_total] = tally(abs(path2.rho - R) < 1e-9, ...
        '1.4 Orbit radius = R_fillet', pass_total, fail_total);
end

% Test 1.5 — Lambda = +1 (CW, right turn from north to east)
if strcmp(path2.type,'orbit')
    [pass_total, fail_total] = tally(path2.lambda == lambda_expected, ...
        sprintf('1.5 Lambda = %+d (CW right turn)', path2.lambda), pass_total, fail_total);
end

% Test 1.6 — Left-turn geometry: north then west → lambda = -1
%   WP1=(0,0) WP2=(400,0) WP3=(400,-400) → turn west from north
WP_lt = [0,0,-100,25; 400,0,-100,25; 400,-400,-100,25];
state_lt.wp_idx = 1; state_lt.fillet_seg = 1;
[~, state_lt2] = path_manager_fillet(WP_lt, [321;0;-100], state_lt, cfg_f);
% Now check orbit direction:
[path_lt, ~] = path_manager_fillet(WP_lt, [321;0;-100], state_lt, cfg_f);
% Re-trigger: actually the state already switched — re-call with orbit state
state_lt3.wp_idx = 1; state_lt3.fillet_seg = 2;
[path_lt2, ~] = path_manager_fillet(WP_lt, [321;1;-100], state_lt3, cfg_f);
if strcmp(path_lt2.type,'orbit')
    [pass_total, fail_total] = tally(path_lt2.lambda == -1, ...
        '1.6 Left turn: lambda = -1 (CCW)', pass_total, fail_total);
else
    [pass_total, fail_total] = tally(false, '1.6 Left turn lambda check', pass_total, fail_total);
end

fprintf('\n');

% =========================================================================
%% Section 2: Fillet state machine transitions
% =========================================================================
fprintf('--- Section 2: Fillet state machine ---\n');

% Waypoints: full rectangle
WP_rect = [0,0,-100,25; 400,0,-100,25; 400,400,-100,25; 0,400,-100,25; 0,0,-100,25];
cfg_r = cfg_f;

% Test 2.1 — UAV on leg 1 (before fillet) → straight-line path toward WP2
state_r.wp_idx = 1; state_r.fillet_seg = 1;
[p, ~] = path_manager_fillet(WP_rect, [200;0;-100], state_r, cfg_r);
[pass_total, fail_total] = tally(strcmp(p.type,'line') && p.q(1) > 0.9, ...
    '2.1 Leg 1: straight line heading north', pass_total, fail_total);

% Test 2.2 — After crossing z2 of WP2 fillet: wp_idx advances to 2
state_adv.wp_idx = 1; state_adv.fillet_seg = 2;
% z2 at WP2 is [400, 80]; cross when pe >= 80 in q_n=[0,1] direction
pos_past_z2 = [400; 85; -100];
[~, state_adv2] = path_manager_fillet(WP_rect, pos_past_z2, state_adv, cfg_r);
[pass_total, fail_total] = tally(state_adv2.wp_idx == 2 && state_adv2.fillet_seg == 1, ...
    '2.2 Past z2: advances to leg 2, fillet_seg=1', pass_total, fail_total);

% Test 2.3 — On leg 2 (WP2→WP3): straight line heading east
state_l2.wp_idx = 2; state_l2.fillet_seg = 1;
[p2, ~] = path_manager_fillet(WP_rect, [400;200;-100], state_l2, cfg_r);
[pass_total, fail_total] = tally(strcmp(p2.type,'line') && p2.q(2) > 0.9, ...
    '2.3 Leg 2: straight line heading east', pass_total, fail_total);

% Test 2.4 — Final leg (WP4→WP5): no fillet, half-plane switching
state_f.wp_idx = 4; state_f.fillet_seg = 1;
[pf, ~] = path_manager_fillet(WP_rect, [200;400;-100], state_f, cfg_r);
[pass_total, fail_total] = tally(strcmp(pf.type,'line'), ...
    '2.4 Final leg: straight-line mode', pass_total, fail_total);

fprintf('\n');

% =========================================================================
%% Section 3: Dubins manager smoke test
% =========================================================================
fprintf('--- Section 3: Dubins manager smoke test ---\n');

rng(0);
cfg_d = cfg_ch10();
cfg_d.T = 60.0;   % Short run: just check no crash or stuck state

state_d.wp_idx    = 1;
state_d.dubins_seg = 1;
state_d.dp        = [];

% Simulate a few steps manually
WP_d = cfg_d.waypoints;
R_d  = cfg_d.R_min;
chi  = 0.0;
pos  = WP_d(1,1:3)';
changed_seg = false;

for k = 1:500
    [path_d, state_d] = path_manager_dubins(WP_d, pos, chi, state_d, cfg_d);
    % Simulate UAV moving along path (simplified: step toward next WP)
    wp_n = WP_d(min(state_d.wp_idx+1, size(WP_d,1)), 1:3)';
    dir  = wp_n - pos;  dm = norm(dir);
    if dm > 1; pos = pos + 5 * dir/dm; end
    if state_d.dubins_seg ~= 1; changed_seg = true; end
    if state_d.wp_idx > 1; break; end
end

% Test 3.1 — Seg state changes (not stuck in seg 1 forever)
[pass_total, fail_total] = tally(changed_seg, ...
    '3.1 Dubins seg state advanced (not stuck in seg 1)', pass_total, fail_total);

% Test 3.2 — path struct is valid
[pass_total, fail_total] = tally(isfield(path_d,'type') && ...
    (strcmp(path_d.type,'line') || strcmp(path_d.type,'orbit')), ...
    '3.2 Dubins path struct is valid', pass_total, fail_total);

% Test 3.3 — fillet manager returns valid path for cfg_ch11
cfg_11 = cfg_ch11();
state_11.wp_idx    = 1;
state_11.fillet_seg = 1;
[path_11, ~] = path_manager_fillet(cfg_11.waypoints, [0;0;-100], state_11, cfg_11);
[pass_total, fail_total] = tally(strcmp(path_11.type,'line'), ...
    '3.3 Fillet manager returns valid path for cfg_ch11', pass_total, fail_total);

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
