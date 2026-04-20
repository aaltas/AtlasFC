% =========================================================================
%  TEST_CH09_GUIDANCE - Unit tests for Chapter 9 guidance functions
% =========================================================================
%  Tests the core path-following algorithms:
%    • follow_straight_line
%    • follow_orbit
%    • waypoint_manager
%
%  Run directly (no simulation, no trim data needed):
%    >> test_ch09_guidance
%
%  Prints PASS/FAIL for each test with tolerance checks.
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 9
% =========================================================================

clc; clear;
fprintf('\n=========================================\n');
fprintf('  AtlasFC  --  Ch09: Guidance Unit Tests\n');
fprintf('=========================================\n\n');

n_pass = 0;
n_fail = 0;

% =========================================================================
%  TEST 1: follow_straight_line — vehicle on path
% =========================================================================
fprintf('TEST 1: follow_straight_line (vehicle on path)\n');
try
    % Vehicle at reference point, heading along path
    path.r = [0; 0; -100];        % Reference point
    path.q = [1/sqrt(2); 1/sqrt(2); 0];  % 45° path, level
    path.Va = 25.0;

    pos = [0; 0; -100];            % On path
    chi = atan2(path.q(2), path.q(1));  % Aligned with path

    [chi_c, h_c, e_py] = follow_straight_line(path, pos, chi);

    % Expected: course command ≈ path heading, cross-track error ≈ 0
    tol_chi = deg2rad(1);
    tol_epy = 0.1;

    if abs(chi_c - chi) < tol_chi && abs(e_py) < tol_epy
        fprintf('  PASS: chi_c=%.4f, e_py=%.6f\n', chi_c, e_py);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: chi_c error=%.4f, e_py=%.6f (expected <%.4f, <%.1f)\n', ...
            abs(chi_c - chi), e_py, tol_chi, tol_epy);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 2: follow_straight_line — vehicle left of path
% =========================================================================
fprintf('\nTEST 2: follow_straight_line (vehicle left of path)\n');
try
    % Vehicle to left of path → should command right turn
    path.r = [100; 0; -100];       % Reference point
    path.q = [1; 0; 0];             % Path along north
    path.Va = 25.0;

    pos = [100; 10; -100];          % 10m left of path
    chi = 0;                        % Heading north

    [chi_c, h_c, e_py] = follow_straight_line(path, pos, chi);

    % Expected: e_py > 0 (left), chi_c < chi (right turn)
    if e_py > 5 && chi_c < chi
        fprintf('  PASS: e_py=%.2f m (left), chi_c=%.4f (turning right)\n', e_py, chi_c);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: e_py=%.2f, chi_c=%.4f (expected e_py>5, chi_c<0)\n', e_py, chi_c);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 3: follow_straight_line — vehicle right of path
% =========================================================================
fprintf('\nTEST 3: follow_straight_line (vehicle right of path)\n');
try
    % Vehicle to right of path → should command left turn
    path.r = [100; 0; -100];
    path.q = [1; 0; 0];
    path.Va = 25.0;

    pos = [100; -10; -100];         % 10m right of path
    chi = 0;

    [chi_c, h_c, e_py] = follow_straight_line(path, pos, chi);

    % Expected: e_py < 0 (right), chi_c > chi (left turn)
    if e_py < -5 && chi_c > chi
        fprintf('  PASS: e_py=%.2f m (right), chi_c=%.4f (turning left)\n', e_py, chi_c);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: e_py=%.2f, chi_c=%.4f (expected e_py<-5, chi_c>0)\n', e_py, chi_c);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 4: follow_orbit — vehicle on orbit
% =========================================================================
fprintf('\nTEST 4: follow_orbit (vehicle on orbit)\n');
try
    % Vehicle on circular orbit
    orbit.c = [200; 200; -100];     % Center
    orbit.rho = 100;                 % 100m radius
    orbit.lambda = 1;                % CCW
    orbit.Va = 25.0;

    % Vehicle on orbit at angle 0°
    pos = [300; 200; -100];          % 100m east of center

    % Heading tangent to circle at this point (north for CCW)
    chi = pi/2;

    [chi_c, h_c, e_r] = follow_orbit(orbit, pos, chi);

    % Expected: radial error ≈ 0
    tol_er = 1.0;
    if abs(e_r) < tol_er
        fprintf('  PASS: e_r=%.4f m (on orbit)\n', e_r);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: e_r=%.4f (expected <%.1f)\n', e_r, tol_er);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 5: follow_orbit — vehicle inside orbit
% =========================================================================
fprintf('\nTEST 5: follow_orbit (vehicle inside orbit)\n');
try
    % Vehicle inside desired orbit → should command outward
    orbit.c = [200; 200; -100];
    orbit.rho = 100;
    orbit.lambda = 1;
    orbit.Va = 25.0;

    pos = [250; 200; -100];          % 50m from center (inside 100m orbit)
    chi = pi/2;

    [chi_c, h_c, e_r] = follow_orbit(orbit, pos, chi);

    % Expected: e_r < 0 (inside)
    if e_r < 0
        fprintf('  PASS: e_r=%.2f m (inside, negative)\n', e_r);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: e_r=%.2f (expected <0)\n', e_r);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 6: follow_orbit — vehicle outside orbit
% =========================================================================
fprintf('\nTEST 6: follow_orbit (vehicle outside orbit)\n');
try
    % Vehicle outside desired orbit → should command inward
    orbit.c = [200; 200; -100];
    orbit.rho = 100;
    orbit.lambda = 1;
    orbit.Va = 25.0;

    pos = [320; 200; -100];          % 120m from center (outside 100m orbit)
    chi = pi/2;

    [chi_c, h_c, e_r] = follow_orbit(orbit, pos, chi);

    % Expected: e_r > 0 (outside)
    if e_r > 0
        fprintf('  PASS: e_r=%.2f m (outside, positive)\n', e_r);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: e_r=%.2f (expected >0)\n', e_r);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 7: waypoint_manager — initial state
% =========================================================================
fprintf('\nTEST 7: waypoint_manager (initial state)\n');
try
    waypoints = [
        0,    0, -100, 25;
        100,  0, -100, 25;
        100, 100, -100, 25;
        0,   100, -100, 25;
    ];

    state = guidance_init(waypoints, struct());
    pos = [0; 0; -100];
    cfg = struct('loop_waypoints', false);

    [path, state_out] = waypoint_manager(waypoints, pos, state, cfg);

    % Expected: path from WP1 to WP2
    if abs(path.r(1) - 0) < 0.1 && abs(path.r(2) - 0) < 0.1 && state_out.wp_idx == 1
        fprintf('  PASS: Start at WP1, path direction correct\n');
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: wp_idx=%d, path.r=[%.1f, %.1f] (expected 1, [0, 0])\n', ...
            state_out.wp_idx, path.r(1), path.r(2));
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 8: waypoint_manager — half-plane switching
% =========================================================================
fprintf('\nTEST 8: waypoint_manager (half-plane switching)\n');
try
    waypoints = [
        0,    0, -100, 25;
        100,  0, -100, 25;
        100, 100, -100, 25;
    ];

    state = guidance_init(waypoints, struct());
    cfg = struct('loop_waypoints', false);

    % Move vehicle across half-plane between WP1 and WP2
    % Half-plane normal points toward WP3, so crossing is when
    % vehicle passes WP2 going toward WP3
    pos = [110; 50; -100];           % Past WP2 toward WP3

    [path, state_out] = waypoint_manager(waypoints, pos, state, cfg);

    % Expected: advanced to WP2
    if state_out.wp_idx == 2
        fprintf('  PASS: Switched to WP2→WP3 leg\n');
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: wp_idx=%d (expected 2)\n', state_out.wp_idx);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  Summary
% =========================================================================
fprintf('\n=========================================\n');
fprintf('  Test Summary: %d PASS, %d FAIL\n', n_pass, n_fail);
fprintf('=========================================\n\n');

if n_fail == 0
    fprintf('SUCCESS: All tests passed.\n\n');
else
    fprintf('FAILURE: %d test(s) failed.\n\n', n_fail);
end
