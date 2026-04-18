% =========================================================================
%  TEST_CH04_FORCES_MOMENTS - Unit tests for Chapter 4 functions
% =========================================================================
%  Tests:
%    T01  gravity_force: level attitude  → fz = m*g, fx=fy=0
%    T02  gravity_force: rolled 90 deg  → fy = m*g, fz≈0
%    T03  gravity_force: output size 3x1
%    T04  lift_drag_coefficients: small alpha → sigma≈0 (linear regime)
%    T05  lift_drag_coefficients: large alpha → sigma≈1 (stalled)
%    T06  lift_drag_coefficients: C_L at alpha=0 equals C_L_0
%    T07  lift_drag_coefficients: C_D > 0 always
%    T08  aerodynamic_forces_moments: zero Va → zero forces
%    T09  aerodynamic_forces_moments: output sizes 3x1 each
%    T10  aerodynamic_forces_moments: symmetric → fy=0 at beta=0, delta_a=0, delta_r=0
%    T11  propulsion_forces_moments: delta_t=0 → near-zero thrust
%    T12  propulsion_forces_moments: delta_t=1 → positive thrust
%    T13  propulsion_forces_moments: thrust increases with delta_t
%    T14  forces_moments: output size 6x1
%    T15  forces_moments: headwind reduces Va below body-frame speed
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 4
% =========================================================================

% --- Auto-path setup (pwd-based, avoids Mac Editor temp-path bug) ---
atlas_root = fullfile(pwd, '..');
addpath(atlas_root);
setup_paths(atlas_root);

clear; clc;
atlas_root = fullfile(pwd, '..');
addpath(atlas_root);
setup_paths(atlas_root);

atlas_root = fullfile(this_dir, '..');
setup_paths(atlas_root);

fprintf('\n========================================\n');
fprintf('  AtlasFC -- Test Suite: Chapter 4\n');
fprintf('========================================\n\n');

params = mav_params();
mg     = params.mass * params.gravity;
TOL    = 1e-6;
pass   = 0;
fail   = 0;

% Common level state
q_lvl = euler_to_quaternion(0, 0, 0);
s0    = [0;0;-100; 25;0;0; q_lvl; 0;0;0];

% =====================================================================
fprintf('--- gravity_force ---\n');

% T01: Level attitude → fz = m*g, fx = fy = 0
fg1 = gravity_force(q_lvl, params);
t01 = abs(fg1(1)) < TOL && abs(fg1(2)) < TOL && abs(fg1(3) - mg) < 0.01;
if t01; pass=pass+1; fprintf('  [PASS] T01: level attitude -> fz = m*g (%.2f N)\n', fg1(3));
else;   fail=fail+1; fprintf('  [FAIL] T01: fg = [%.3f, %.3f, %.3f]\n', fg1(1), fg1(2), fg1(3)); end

% T02: Rolled 90 deg → fy = m*g, fz ≈ 0
q_roll90 = euler_to_quaternion(pi/2, 0, 0);
fg2      = gravity_force(q_roll90, params);
t02 = abs(fg2(2) - mg) < 0.1 && abs(fg2(3)) < 0.1;
if t02; pass=pass+1; fprintf('  [PASS] T02: 90 deg roll -> fy = m*g (%.2f N)\n', fg2(2));
else;   fail=fail+1; fprintf('  [FAIL] T02: fg = [%.3f, %.3f, %.3f]\n', fg2(1), fg2(2), fg2(3)); end

% T03: Output size 3x1
t03 = isequal(size(fg1), [3,1]);
if t03; pass=pass+1; fprintf('  [PASS] T03: output size is 3x1\n');
else;   fail=fail+1; fprintf('  [FAIL] T03: size = %dx%d\n', size(fg1,1), size(fg1,2)); end

% =====================================================================
fprintf('\n--- lift_drag_coefficients ---\n');

% T04: Small alpha → sigma ≈ 0 (linear regime)
[CL4, CD4] = lift_drag_coefficients(0.05, params);
CL_lin4 = params.C_L_0 + params.C_L_alpha * 0.05;
t04 = abs(CL4 - CL_lin4) < 0.01;
if t04; pass=pass+1; fprintf('  [PASS] T04: small alpha -> CL ≈ linear (%.4f vs %.4f)\n', CL4, CL_lin4);
else;   fail=fail+1; fprintf('  [FAIL] T04: CL=%.4f, expected≈%.4f\n', CL4, CL_lin4); end

% T05: Large alpha → CL near flat-plate model
[CL5, ~] = lift_drag_coefficients(pi/2, params);
CL_fp5   = 2 * sign(pi/2) * sin(pi/2)^2 * cos(pi/2);   % ≈ 0 at 90 deg
t05 = abs(CL5 - CL_fp5) < 0.1;
if t05; pass=pass+1; fprintf('  [PASS] T05: large alpha -> CL near flat-plate (%.4f)\n', CL5);
else;   fail=fail+1; fprintf('  [FAIL] T05: CL=%.4f (expected flat-plate≈%.4f)\n', CL5, CL_fp5); end

% T06: alpha=0 → C_L ≈ C_L_0
[CL6, ~] = lift_drag_coefficients(0, params);
t06 = abs(CL6 - params.C_L_0) < 0.05;
if t06; pass=pass+1; fprintf('  [PASS] T06: alpha=0 -> CL ≈ C_L_0 (%.4f)\n', CL6);
else;   fail=fail+1; fprintf('  [FAIL] T06: CL=%.4f, C_L_0=%.4f\n', CL6, params.C_L_0); end

% T07: C_D always positive
[~, CD7a] = lift_drag_coefficients( 0.3, params);
[~, CD7b] = lift_drag_coefficients(-0.3, params);
t07 = CD7a > 0 && CD7b > 0;
if t07; pass=pass+1; fprintf('  [PASS] T07: C_D > 0 for +/- alpha\n');
else;   fail=fail+1; fprintf('  [FAIL] T07: CD(+0.3)=%.4f, CD(-0.3)=%.4f\n', CD7a, CD7b); end

% =====================================================================
fprintf('\n--- aerodynamic_forces_moments ---\n');

% T08: Zero Va → zero forces
[fa8, ma8] = aerodynamic_forces_moments(0, 0, 0, 0, 0, 0, 0, 0, 0, params);
t08 = norm(fa8) < TOL && norm(ma8) < TOL;
if t08; pass=pass+1; fprintf('  [PASS] T08: Va=0 -> zero aero forces\n');
else;   fail=fail+1; fprintf('  [FAIL] T08: |fa|=%.2e, |ma|=%.2e\n', norm(fa8), norm(ma8)); end

% T09: Output sizes 3x1
[fa9, ma9] = aerodynamic_forces_moments(25, 0.1, 0, 0, 0, 0, 0, 0, 0, params);
t09 = isequal(size(fa9), [3,1]) && isequal(size(ma9), [3,1]);
if t09; pass=pass+1; fprintf('  [PASS] T09: output sizes are 3x1\n');
else;   fail=fail+1; fprintf('  [FAIL] T09: wrong output sizes\n'); end

% T10: Symmetric aircraft at beta=0, no lateral controls → fy ≈ 0
[fa10, ~] = aerodynamic_forces_moments(25, 0.1, 0, 0, 0, 0, 0, 0, 0, params);
t10 = abs(fa10(2)) < 0.01;
if t10; pass=pass+1; fprintf('  [PASS] T10: symmetric -> fy=0 at beta=0 (fy=%.4f)\n', fa10(2));
else;   fail=fail+1; fprintf('  [FAIL] T10: fy=%.4f (expected 0)\n', fa10(2)); end

% =====================================================================
fprintf('\n--- propulsion_forces_moments ---\n');

Va_test = 25;   % m/s

% T11: delta_t = 0 → very small thrust
[fp11, ~] = propulsion_forces_moments(Va_test, 0.0, params);
t11 = fp11(1) < 5.0;   % some small value due to windmilling
if t11; pass=pass+1; fprintf('  [PASS] T11: delta_t=0 -> near-zero thrust (%.2f N)\n', fp11(1));
else;   fail=fail+1; fprintf('  [FAIL] T11: thrust=%.2f N at delta_t=0\n', fp11(1)); end

% T12: delta_t = 1 → positive thrust
[fp12, ~] = propulsion_forces_moments(Va_test, 1.0, params);
t12 = fp12(1) > 0;
if t12; pass=pass+1; fprintf('  [PASS] T12: delta_t=1 -> positive thrust (%.2f N)\n', fp12(1));
else;   fail=fail+1; fprintf('  [FAIL] T12: thrust=%.2f N at delta_t=1\n', fp12(1)); end

% T13: Thrust monotonically increases with delta_t
[fp13a, ~] = propulsion_forces_moments(Va_test, 0.3, params);
[fp13b, ~] = propulsion_forces_moments(Va_test, 0.7, params);
t13 = fp13b(1) > fp13a(1);
if t13; pass=pass+1; fprintf('  [PASS] T13: thrust increases with delta_t (%.1f -> %.1f N)\n', fp13a(1), fp13b(1));
else;   fail=fail+1; fprintf('  [FAIL] T13: T(0.3)=%.1f, T(0.7)=%.1f\n', fp13a(1), fp13b(1)); end

% =====================================================================
fprintf('\n--- forces_moments ---\n');

w_zero = zeros(3,1);

% T14: Output size 6x1
[fm14, ~, ~, ~] = forces_moments(s0, [0;0;0;0.5], w_zero, w_zero, params);
t14 = isequal(size(fm14), [6,1]);
if t14; pass=pass+1; fprintf('  [PASS] T14: output size is 6x1\n');
else;   fail=fail+1; fprintf('  [FAIL] T14: wrong output size\n'); end

% T15: Headwind reduces Va below body-frame speed
[~, Va15_no_wind, ~, ~] = forces_moments(s0, [0;0;0;0.5], [0;0;0],  w_zero, params);
[~, Va15_wind,    ~, ~] = forces_moments(s0, [0;0;0;0.5], [5;0;0],  w_zero, params);  % 5 m/s headwind (north)
t15 = Va15_wind < Va15_no_wind;
if t15; pass=pass+1; fprintf('  [PASS] T15: headwind reduces Va (%.2f -> %.2f m/s)\n', Va15_no_wind, Va15_wind);
else;   fail=fail+1; fprintf('  [FAIL] T15: Va_no_wind=%.2f, Va_wind=%.2f\n', Va15_no_wind, Va15_wind); end

% =====================================================================
fprintf('\n========================================\n');
fprintf('  Results: %d / %d tests passed\n', pass, pass+fail);
if fail == 0
    fprintf('  *** ALL TESTS PASSED ***\n');
else
    fprintf('  *** %d TEST(S) FAILED ***\n', fail);
end
fprintf('========================================\n\n');
