% =========================================================================
%  TEST_CH03_KINEMATICS - Unit tests for Chapter 3 kinematic functions
% =========================================================================
%  Tests:
%    T01  euler_kinematics  zero rates → zero Euler rates
%    T02  euler_kinematics  pure roll p → only phi_dot
%    T03  euler_kinematics  pure pitch q at level → only theta_dot
%    T04  euler_kinematics  analytic formula match (non-trivial attitude)
%    T05  quaternion_kinematics  zero omega → zero e_dot
%    T06  quaternion_kinematics  output size is 4x1
%    T07  quaternion_kinematics  Xi formula exact match
%    T08  mav_dynamics  output size is 13x1
%    T09  mav_dynamics  quaternion renormalized after step
%    T10  mav_dynamics  free fall w_dot ≈ g (1 step from rest)
%    T11  mav_dynamics  no forces → body velocities unchanged
%    T12  mav_dynamics  no moments → angular rates unchanged
%    T13  mav_dynamics  forward flight moves north (pn > 0)
%    T14  mav_dynamics  RK4 convergence: smaller dt more accurate
%    T15  euler_kinematics  gimbal lock near theta = 90 deg
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 3
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
fprintf('  AtlasFC -- Test Suite: Chapter 3\n');
fprintf('========================================\n\n');

params = mav_params();
g      = params.gravity;
m      = params.mass;
TOL    = 1e-10;
pass   = 0;
fail   = 0;

% Shorthand for pass/fail reporting
function [p, f] = check(label, cond, p, f)
    if cond
        fprintf('  [PASS] %s\n', label);
        p = p + 1;
    else
        fprintf('  [FAIL] %s\n', label);
        f = f + 1;
    end
end

% Common level hover state
q_lvl = euler_to_quaternion(0, 0, 0);
s0    = [0;0;0; 0;0;0; q_lvl; 0;0;0];   % 13x1

% =====================================================================
fprintf('--- euler_kinematics ---\n');

% T01: Zero rates → zero Euler rates
[pd1, td1, yd1] = euler_kinematics(0, 0, 0,  0, 0, 0);
[pass, fail] = check('T01: zero rates -> zero Euler rates', ...
    abs(pd1)<TOL && abs(td1)<TOL && abs(yd1)<TOL, pass, fail);

% T02: Pure roll p at level → only phi_dot nonzero
[pd2, td2, yd2] = euler_kinematics(0, 0, 0,  1, 0, 0);
[pass, fail] = check('T02: pure p -> only phi_dot = 1', ...
    abs(pd2-1)<TOL && abs(td2)<TOL && abs(yd2)<TOL, pass, fail);

% T03: Pure pitch q at level → only theta_dot
[pd3, td3, yd3] = euler_kinematics(0, 0, 0,  0, 1, 0);
[pass, fail] = check('T03: pure q -> only theta_dot = 1', ...
    abs(pd3)<TOL && abs(td3-1)<TOL && abs(yd3)<TOL, pass, fail);

% T04: Non-trivial attitude — compare with hand-built T matrix
phi4=0.3; theta4=0.4; p4=0.1; q4=0.2; r4=0.15;
[pd4, td4, yd4] = euler_kinematics(phi4, theta4, 0, p4, q4, r4);
T4 = [1, sin(phi4)*tan(theta4), cos(phi4)*tan(theta4);
      0, cos(phi4),            -sin(phi4);
      0, sin(phi4)/cos(theta4), cos(phi4)/cos(theta4)];
ref4 = T4 * [p4; q4; r4];
[pass, fail] = check('T04: analytic T-matrix match', ...
    norm([pd4;td4;yd4] - ref4) < TOL, pass, fail);

% =====================================================================
fprintf('\n--- quaternion_kinematics ---\n');

% T05: Zero omega → zero e_dot
edot5 = quaternion_kinematics([1;0;0;0], [0;0;0]);
[pass, fail] = check('T05: zero omega -> zero e_dot', norm(edot5)<TOL, pass, fail);

% T06: Output size 4x1
edot6 = quaternion_kinematics([1;0;0;0], [0.1;0.2;0.3]);
[pass, fail] = check('T06: output size is 4x1', isequal(size(edot6),[4,1]), pass, fail);

% T07: Xi formula exact
e7 = [0.9;0.2;0.1;0.3]; e7 = e7/norm(e7);
om7 = [0.1; 0.3; -0.2];
edot7 = quaternion_kinematics(e7, om7);
e0_=e7(1); e1_=e7(2); e2_=e7(3); e3_=e7(4);
Xi7 = [-e1_,-e2_,-e3_; e0_,-e3_,e2_; e3_,e0_,-e1_; -e2_,e1_,e0_];
ref7 = 0.5 * Xi7 * om7;
[pass, fail] = check('T07: Xi formula exact match', norm(edot7-ref7)<TOL, pass, fail);

% =====================================================================
fprintf('\n--- mav_dynamics ---\n');

fm0 = zeros(6,1);   % no forces, no moments

% T08: Output size 13x1
s8 = mav_dynamics(s0, fm0, params, 0.01);
[pass, fail] = check('T08: output size is 13x1', isequal(size(s8),[13,1]), pass, fail);

% T09: Quaternion renormalized after step
s9_init = s0;
s9_init(7:10) = [1.05; 0.02; 0.02; 0.02];   % deliberately off-norm
s9 = mav_dynamics(s9_init, fm0, params, 0.01);
[pass, fail] = check('T09: quaternion renormalized', abs(norm(s9(7:10))-1)<1e-12, pass, fail);

% T10: Free fall from rest → w_dot ≈ g (gravity in body z when level)
fm10 = [0; 0; m*g; 0; 0; 0];
s10  = mav_dynamics(s0, fm10, params, 0.001);
w_dot_approx = (s10(6) - s0(6)) / 0.001;
[pass, fail] = check(sprintf('T10: w_dot = g in free fall (%.4f vs %.4f)', w_dot_approx, g), ...
    abs(w_dot_approx - g) < 0.01, pass, fail);

% T11: No forces → body velocities unchanged
s11 = mav_dynamics(s0, fm0, params, 0.01);
[pass, fail] = check('T11: no forces -> velocities unchanged', ...
    norm(s11(4:6) - s0(4:6)) < TOL, pass, fail);

% T12: No moments → angular rates unchanged
[pass, fail] = check('T12: no moments -> angular rates unchanged', ...
    norm(s11(11:13) - s0(11:13)) < TOL, pass, fail);

% T13: Forward flight → pn increases
s13_init = [0;0;0; 10;0;0; q_lvl; 0;0;0];
s13 = mav_dynamics(s13_init, fm0, params, 0.01);
[pass, fail] = check(sprintf('T13: forward flight -> pn > 0 (pn=%.4f)', s13(1)), ...
    s13(1) > 0, pass, fail);

% T14: RK4 convergence — smaller dt gives less error
fm14 = [0; 0; m*g; 0; 0; 0];
s14_coarse = mav_dynamics(s0, fm14, params, 0.1);
s14_fine   = mav_dynamics(s0, fm14, params, 0.001);
err_coarse = abs(s14_coarse(6) - g*0.1);
err_fine   = abs(s14_fine(6)   - g*0.001);
[pass, fail] = check('T14: RK4 convergence (smaller dt more accurate)', ...
    err_fine < err_coarse, pass, fail);

% T15: Gimbal lock near theta = 90 deg → psi_dot blows up
[~, ~, yd15] = euler_kinematics(0, pi/2 - 1e-4, 0,  0, 0, 0.1);
[pass, fail] = check(sprintf('T15: gimbal lock detected (psi_dot=%.1f >> 1)', yd15), ...
    abs(yd15) > 100, pass, fail);

% =====================================================================
fprintf('\n========================================\n');
fprintf('  Results: %d / %d tests passed\n', pass, pass+fail);
if fail == 0
    fprintf('  *** ALL TESTS PASSED ***\n');
else
    fprintf('  *** %d TEST(S) FAILED ***\n', fail);
end
fprintf('========================================\n\n');
