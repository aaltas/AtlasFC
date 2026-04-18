% =========================================================================
%  TEST_CH05_TRIM_LINEARIZATION - Unit tests for Chapter 5
% =========================================================================
%  Run from the tests/ directory:
%    >> cd .../AtlasFC/tests
%    >> test_ch05_trim_linearization
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 5
% =========================================================================

% --- Auto-path setup (pwd-based, avoids Mac Editor temp-path bug) ---
atlas_root = fullfile(pwd, '..');
addpath(atlas_root);
setup_paths(atlas_root);

clear; clc;
atlas_root = fullfile(pwd, '..');
addpath(atlas_root);
setup_paths(atlas_root);

fprintf('\n========================================\n');
fprintf('  AtlasFC  --  Ch05 Unit Tests\n');
fprintf('========================================\n\n');

params = mav_params();
Va_s   = 25.0;

% Compute trim once — used by all tests
[x_trim, u_trim, info] = compute_trim(Va_s, 0, inf, params);
[A, B]                 = linearize(x_trim, u_trim, params);
[sys_lon, sys_lat]     = decouple_models(A, B);
tf_s                   = transfer_functions(x_trim, u_trim, params);

pass = 0;  fail = 0;

%% === TRIM TESTS ===
fprintf('=== TRIM TESTS ===\n');

% T01
ok = info.converged;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T01: Trim optimizer converged (cost<1e-6)\n');

% T02
Va_actual = sqrt(x_trim(4)^2 + x_trim(5)^2 + x_trim(6)^2);
ok = abs(Va_actual - Va_s) < 0.1;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T02: Trim Va within 0.1 m/s of request\n');

% T03
alpha_t = atan2(x_trim(6), x_trim(4));
ok = alpha_t > 0 && rad2deg(alpha_t) < 15;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T03: Trim alpha in [0, 15] deg\n');

% T04
ok = u_trim(4) > 0.1 && u_trim(4) < 0.9;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T04: Trim throttle in [0.1, 0.9]\n');

% T05
ok = abs(u_trim(2)) < 1e-6;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T05: Trim delta_a = 0\n');

% T06
ok = abs(x_trim(7)) < 1e-6;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T06: Trim phi = 0\n');

% T07
e_q = euler_to_quaternion(x_trim(7), x_trim(8), x_trim(9));
x13 = [x_trim(1:6); e_q; x_trim(10:12)];
[fm, ~, ~, ~] = forces_moments(x13, u_trim, zeros(3,1), zeros(3,1), params);
accel_res = sqrt((fm(1)/params.mass)^2 + (fm(3)/params.mass)^2 + (fm(5)/params.Jy)^2);
ok = accel_res < 0.1;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T07: Trim residual accelerations < 0.1 m/s^2 (got %.4f)\n', accel_res);

%% === LINEARIZATION TESTS ===
fprintf('\n=== LINEARIZATION TESTS ===\n');

% T08
ok = all(size(A) == [12,12]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T08: Full A matrix is 12x12\n');

% T09
ok = all(size(B) == [12,4]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T09: Full B matrix is 12x4\n');

% T10
ok = all(size(sys_lon.A) == [5,5]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T10: Longitudinal A is 5x5\n');

% T11
ok = all(size(sys_lat.A) == [5,5]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T11: Lateral A is 5x5\n');

% T12
eig_lon = eig(sys_lon.A);
ok = sum(real(eig_lon) < 0) >= 2;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T12: Longitudinal has >= 2 stable modes\n');

% T13
eig_lat = eig(sys_lat.A);
ok = sum(real(eig_lat) < 0) >= 2;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T13: Lateral has >= 2 stable modes\n');

% T14
ok = all(isfinite(A(:)));
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T14: A matrix has all finite entries\n');

%% === TRANSFER FUNCTION TESTS ===
fprintf('\n=== TRANSFER FUNCTION TESTS ===\n');

% T15
ok = tf_s.a_phi1 > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T15: a_phi1 > 0 (roll damping positive, got %.4f)\n', tf_s.a_phi1);

% T16
ok = tf_s.a_phi2 > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T16: a_phi2 > 0 (aileron authority positive, got %.4f)\n', tf_s.a_phi2);

% T17
ok = tf_s.a_beta1 > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T17: a_beta1 > 0 (stable sideslip, got %.4f)\n', tf_s.a_beta1);

% T18
ok = tf_s.a_theta1 > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T18: a_theta1 > 0 (pitch rate damping, got %.4f)\n', tf_s.a_theta1);

% T19
ok = tf_s.a_theta2 > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T19: a_theta2 > 0 (pitch stiffness, got %.4f)\n', tf_s.a_theta2);

% T20
ok = tf_s.a_theta3 < 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T20: a_theta3 < 0 (elevator sign convention, got %.4f)\n', tf_s.a_theta3);

% T21
ok = tf_s.a_V2 > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T21: a_V2 > 0 (throttle increases airspeed, got %.4f)\n', tf_s.a_V2);

% T22
ok = abs(tf_s.Va_star - Va_s) < 0.1;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T22: tf.Va_star matches requested Va\n');

%% === SUMMARY ===
total = pass + fail;
fprintf('\n========================================\n');
fprintf('  Results: %d / %d tests passed\n', pass, total);
if fail == 0
    fprintf('  ALL TESTS PASSED.\n');
else
    fprintf('  %d TEST(S) FAILED.\n', fail);
end
fprintf('========================================\n\n');
