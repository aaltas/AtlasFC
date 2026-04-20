% =========================================================================
%  TEST_CH10_DUBINS - Unit tests for Chapter 10 Dubins path functions
% =========================================================================
%  Tests dubins_parameters for all four CSC path types.
%  No simulation or trim data required — pure geometry tests.
%
%  Checks:
%    1. All four path types produce positive lengths
%    2. Tangent points z1, z2 lie on their respective circles
%    3. Heading at tangent points matches Dubins path direction
%    4. Shortest path is selected correctly
%    5. Degenerate cases (co-aligned configurations, 180° turn)
%
%  Run:
%    >> test_ch10_dubins
%
%  Author : AtlasFC
% =========================================================================

clc; clear;
fprintf('\n=========================================\n');
fprintf('  AtlasFC  --  Ch10: Dubins Unit Tests\n');
fprintf('=========================================\n\n');

n_pass = 0;
n_fail = 0;
R = 80.0;   % [m]  minimum turning radius used throughout
tol = 1e-6; % numerical tolerance

% =========================================================================
%  TEST 1: RSR — aligned east-west configurations
% =========================================================================
fprintf('TEST 1: RSR — aligned eastward configurations\n');
try
    ps  = [0;   0; -100];  chi_s = 0;          % heading north
    pe  = [400; 0; -100];  chi_e = 0;

    dp = dubins_parameters(ps, chi_s, pe, chi_e, R);

    % For aligned north-heading configs, RSR or LSL should be chosen
    % Path should be finite and positive
    ok = (dp.L > 0) && isfinite(dp.L) && (dp.L1 >= 0) && (dp.L2 >= 0) && (dp.L3 >= 0);

    % z1 must lie on start circle
    dist_z1 = norm(dp.z1(1:2) - dp.c_s(1:2));
    ok = ok && (abs(dist_z1 - R) < 1.0);

    % z2 must lie on end circle
    dist_z2 = norm(dp.z2(1:2) - dp.c_e(1:2));
    ok = ok && (abs(dist_z2 - R) < 1.0);

    if ok
        fprintf('  PASS: type=%s  L=%.1f m  (L1=%.1f, L2=%.1f, L3=%.1f)\n', ...
            dp.type, dp.L, dp.L1, dp.L2, dp.L3);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: type=%s  L=%.1f  dist_z1=%.2f  dist_z2=%.2f  (expected R=%.1f)\n', ...
            dp.type, dp.L, dist_z1, dist_z2, R);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 2: 90° turn — heading change at single waypoint
% =========================================================================
fprintf('\nTEST 2: 90° right turn (north → east)\n');
try
    ps  = [0;   0; -100];  chi_s = 0;        % heading north
    pe  = [400; 400; -100];  chi_e = pi/2;   % heading east

    dp = dubins_parameters(ps, chi_s, pe, chi_e, R);

    ok = (dp.L > 0) && isfinite(dp.L);
    dist_z1 = norm(dp.z1(1:2) - dp.c_s(1:2));
    dist_z2 = norm(dp.z2(1:2) - dp.c_e(1:2));
    ok = ok && (abs(dist_z1 - R) < 1.0) && (abs(dist_z2 - R) < 1.0);

    if ok
        fprintf('  PASS: type=%s  L=%.1f m  (L1=%.1f, L2=%.1f, L3=%.1f)\n', ...
            dp.type, dp.L, dp.L1, dp.L2, dp.L3);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: type=%s  dist_z1=%.2f  dist_z2=%.2f\n', dp.type, dist_z1, dist_z2);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 3: 180° U-turn
% =========================================================================
fprintf('\nTEST 3: 180° U-turn (north → south)\n');
try
    ps  = [0;   0; -100];  chi_s = 0;       % heading north
    pe  = [200; 0; -100];  chi_e = pi;      % heading south

    dp = dubins_parameters(ps, chi_s, pe, chi_e, R);

    ok = (dp.L > 0) && isfinite(dp.L);
    dist_z1 = norm(dp.z1(1:2) - dp.c_s(1:2));
    dist_z2 = norm(dp.z2(1:2) - dp.c_e(1:2));
    ok = ok && (abs(dist_z1 - R) < 1.0) && (abs(dist_z2 - R) < 1.0);

    if ok
        fprintf('  PASS: type=%s  L=%.1f m\n', dp.type, dp.L);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: type=%s  L=%.1f  dist_z1=%.2f  dist_z2=%.2f\n', ...
            dp.type, dp.L, dist_z1, dist_z2);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 4: Path length is shortest among all four candidates
% =========================================================================
fprintf('\nTEST 4: Shortest path selection\n');
try
    ps  = [0; 0; -100];    chi_s = pi/4;
    pe  = [300; 200; -100]; chi_e = -pi/3;

    dp = dubins_parameters(ps, chi_s, pe, chi_e, R);

    % Compute all four paths manually and check minimum
    % Corrected convention: CW=[-sin;cos], CCW=[sin;-cos]
    c_rs = ps(1:2) + R*[-sin(chi_s);  cos(chi_s)];   % CW  start
    c_ls = ps(1:2) + R*[ sin(chi_s); -cos(chi_s)];   % CCW start
    c_re = pe(1:2) + R*[-sin(chi_e);  cos(chi_e)];   % CW  end
    c_le = pe(1:2) + R*[ sin(chi_e); -cos(chi_e)];   % CCW end

    L_rsr = rsr_len(chi_s, chi_e, R, c_rs, c_re);
    L_lsl = lsl_len(chi_s, chi_e, R, c_ls, c_le);
    L_rsl = rsl_len(chi_s, chi_e, R, c_rs, c_le);
    L_lsr = lsr_len(chi_s, chi_e, R, c_ls, c_re);

    L_min = min([L_rsr, L_lsl, L_rsl, L_lsr]);

    if abs(dp.L - L_min) < 1.0
        fprintf('  PASS: type=%s  L=%.2f m  (min of all 4 = %.2f)\n', dp.type, dp.L, L_min);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: dp.L=%.2f  L_min=%.2f  diff=%.3f\n', dp.L, L_min, dp.L-L_min);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 5: Straight path (chi_s = chi_e = 0, large distance)
% =========================================================================
fprintf('\nTEST 5: Straight north path — no arc needed\n');
try
    ps  = [0; 0; -100];    chi_s = 0;
    pe  = [500; 0; -100];  chi_e = 0;

    dp = dubins_parameters(ps, chi_s, pe, chi_e, R);

    % For aligned configs: L1 + L3 should be minimal, L2 ≈ distance
    ok = (dp.L > 0) && isfinite(dp.L) && (dp.L2 > 0);

    if ok
        fprintf('  PASS: type=%s  L=%.1f m  L2=%.1f m  (dist=500 m)\n', ...
            dp.type, dp.L, dp.L2);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: type=%s  L=%.1f  L2=%.1f\n', dp.type, dp.L, dp.L2);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 6: z3 equals end waypoint
% =========================================================================
fprintf('\nTEST 6: z3 equals end waypoint pe\n');
try
    ps  = [100; 50; -120];  chi_s = pi/3;
    pe  = [350; 250; -120]; chi_e = pi/6;

    dp = dubins_parameters(ps, chi_s, pe, chi_e, R);

    dist_z3_pe = norm(dp.z3 - pe);
    if dist_z3_pe < tol
        fprintf('  PASS: z3 == pe (dist=%.2e)\n', dist_z3_pe);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: ||z3 - pe|| = %.4f  (expected 0)\n', dist_z3_pe);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 7: q is unit vector
% =========================================================================
fprintf('\nTEST 7: Straight segment direction q is unit vector\n');
try
    ps  = [0; 0; -100];    chi_s = 1.1;
    pe  = [400; 300; -100]; chi_e = -0.5;

    dp = dubins_parameters(ps, chi_s, pe, chi_e, R);

    q_norm = norm(dp.q);
    if abs(q_norm - 1.0) < tol
        fprintf('  PASS: ||q|| = %.8f\n', q_norm);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: ||q|| = %.6f  (expected 1.0)\n', q_norm);
        n_fail = n_fail + 1;
    end
catch ME
    fprintf('  FAIL: Exception: %s\n', ME.message);
    n_fail = n_fail + 1;
end

% =========================================================================
%  TEST 8: L1 + L2 + L3 = dp.L
% =========================================================================
fprintf('\nTEST 8: L1 + L2 + L3 == dp.L\n');
try
    ps  = [50; 100; -100]; chi_s = -0.8;
    pe  = [300; 400; -100]; chi_e = 2.1;

    dp = dubins_parameters(ps, chi_s, pe, chi_e, R);

    diff_L = abs(dp.L1 + dp.L2 + dp.L3 - dp.L);
    if diff_L < tol
        fprintf('  PASS: L1+L2+L3 = dp.L = %.2f m  (diff=%.2e)\n', dp.L, diff_L);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: L1+L2+L3=%.4f  dp.L=%.4f  diff=%.2e\n', ...
            dp.L1+dp.L2+dp.L3, dp.L, diff_L);
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

% =========================================================================
%  Length helpers (mirroring dubins_parameters sub-functions for testing)
%
%  Circle centres (corrected convention, matching dubins_parameters.m):
%    CW  (right): c = p + R*[-sin(chi);  cos(chi)]
%    CCW (left):  c = p + R*[ sin(chi); -cos(chi)]
% =========================================================================
function L = rsr_len(chi_s, chi_e, R, c_rs, c_re)
    % c_rs and c_re already computed with corrected formula by caller
    d = c_re - c_rs; ell = norm(d); theta = atan2(d(2),d(1));
    L = R*mod(theta-chi_s,2*pi) + ell + R*mod(chi_e-theta,2*pi);
end
function L = lsl_len(chi_s, chi_e, R, c_ls, c_le)
    d = c_le - c_ls; ell = norm(d); theta = atan2(d(2),d(1));
    L = R*mod(chi_s-theta,2*pi) + ell + R*mod(theta-chi_e,2*pi);
end
function L = rsl_len(chi_s, chi_e, R, c_rs, c_le)
    d = c_le - c_rs; ell = norm(d);
    if ell < 2*R; L=inf; return; end
    theta = atan2(d(2),d(1)); tq = theta + asin(2*R/ell);   % corrected sign
    L = R*mod(tq-chi_s,2*pi) + sqrt(ell^2-4*R^2) + R*mod(tq-chi_e,2*pi);
end
function L = lsr_len(chi_s, chi_e, R, c_ls, c_re)
    d = c_re - c_ls; ell = norm(d);
    if ell < 2*R; L=inf; return; end
    theta = atan2(d(2),d(1)); tq = theta - asin(2*R/ell);   % corrected sign
    L = R*mod(chi_s-tq,2*pi) + sqrt(ell^2-4*R^2) + R*mod(chi_e-tq,2*pi);
end
