% =========================================================================
%  TEST_CH08_ESTIMATOR - Unit tests for Chapter 8 EKF
% =========================================================================
%  Tests:
%    T01  nav_equations  xdot is 13x1
%    T02  nav_equations  at trim, ṗ ≈ [Va; 0; 0] in NED
%    T03  nav_equations  A is 13x13
%    T04  nav_equations  B is 13x6
%    T05  nav_equations  A numerical check (finite diff) — block ∂v̇/∂b
%    T06  nav_equations  B check: ∂v̇/∂y_accel = I
%    T07  skew3          skew3(v)*u = cross(v,u)
%    T08  ekf_params     Q is 13x13 positive diagonal
%    T09  ekf_params     R_gps is 4x4 positive diagonal
%    T10  ekf_init       x_hat equals input, quaternion normalized
%    T11  ekf_propagate  output x_hat is 13x1
%    T12  ekf_propagate  quaternion stays normalized after propagation
%    T13  ekf_propagate  P remains symmetric positive definite
%    T14  ekf_update     state update correct direction (baro example)
%    T15  ekf_update     P decreases after measurement update
%    T16  ekf_update     quaternion normalized after update
%    T17  meas_baro      h_pred = -pd, H(3)=-1
%    T18  meas_pitot     h_pred = Va, H has correct velocity entries
%    T19  meas_mag       h_pred is scalar angle, H(7:10) nonzero
%    T20  meas_gps       h_pred is 4x1, rows 1-2 = position
%    T21  meas_gps       H row 3 = pitot-like velocity Jacobian
%    T22  ekf            output x_hat is 13x1, P is 13x13
%    T23  ekf            quaternion stays normalized through full EKF step
%    T24  ekf            after 100 GPS updates, position error < 1m
%
%  Author : AtlasFC  |  Ref: Beard & McLain Ch. 8
% =========================================================================

% --- Auto-path setup ---
atlas_root = fullfile(pwd, '..');
addpath(atlas_root);
setup_paths(atlas_root);

clear; clc; rng(42);
atlas_root = fullfile(pwd, '..');
addpath(atlas_root);
setup_paths(atlas_root);

fprintf('\n========================================\n');
fprintf('  AtlasFC  --  Ch08 Unit Tests\n');
fprintf('========================================\n\n');

% --- Setup ---
params  = mav_params();
sparams = sensor_params();
ep      = ekf_params(sparams);

[x_trim, u_trim, ~] = compute_trim(25.0, 0, inf, params);
e_q = euler_to_quaternion(x_trim(7), x_trim(8), x_trim(9));
x13 = [x_trim(1:6); e_q; x_trim(10:12)];
x13(3) = -100;

[fm, ~, ~, ~] = forces_moments(x13, u_trim, zeros(3,1), zeros(3,1), params);

u_imu = [x13(11:13); fm(1:3)/params.mass - ...
         quaternion_to_rotation(x13(7:10))*[0;0;params.gravity]];
% At trim: gyro = [p;q;r] = 0, accel = specific force

pass=0; fail=0;
TOL_loose = 1e-3;

% -----------------------------------------------------------------------
%% NAV_EQUATIONS TESTS
% -----------------------------------------------------------------------
fprintf('=== NAV_EQUATIONS TESTS ===\n');

% T01
[xdot, A, B] = nav_equations(x13, u_imu, params);
ok = isequal(size(xdot), [13,1]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T01: xdot is 13x1\n');

% T02: at trim, ṗ ≈ [Va;0;0] in NED (trim flies straight ahead)
Va_trim = sqrt(x13(4)^2+x13(5)^2+x13(6)^2);
p_dot_NED = xdot(1:3);
ok = abs(p_dot_NED(1) - Va_trim) < 1.0 && abs(p_dot_NED(2)) < 0.5;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T02: trim ṗ_NED = [%.2f, %.2f, %.2f] m/s\n', p_dot_NED(1), p_dot_NED(2), p_dot_NED(3));

% T03
ok = isequal(size(A), [13,13]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T03: A is 13x13\n');

% T04
ok = isequal(size(B), [13,6]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T04: B is 13x6\n');

% T05: Numerical check of A — ∂v̇/∂b (indices 4:6, 11:13) = -skew(v)
eps_fd = 1e-6;
A_num = zeros(13,13);
for j = 11:13
    dx = zeros(13,1); dx(j) = eps_fd;
    xdot_p = nav_equations(x13+dx, u_imu, params);
    xdot_m = nav_equations(x13-dx, u_imu, params);
    A_num(:,j) = (xdot_p - xdot_m) / (2*eps_fd);
end
err_vb = norm(A(4:6,11:13) - A_num(4:6,11:13), 'fro');
ok = err_vb < 1e-5;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T05: ∂v̇/∂b Jacobian vs finite diff (err=%.2e)\n', err_vb);

% T06: B check — ∂v̇/∂y_accel = I (indices 4:6, 4:6 of B)
ok = norm(B(4:6, 4:6) - eye(3), 'fro') < 1e-12;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T06: B(4:6,4:6) = I (accel → velocity)\n');

% -----------------------------------------------------------------------
%% SKEW3 TEST
% -----------------------------------------------------------------------
fprintf('\n=== SKEW3 TEST ===\n');

% T07
a = [1.2; -0.5; 3.0];  b_vec = [0.1; 2.0; -1.1];
ok = norm(skew3(a)*b_vec - cross(a,b_vec)) < 1e-12;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T07: skew3(v)*u = cross(v,u)\n');

% -----------------------------------------------------------------------
%% EKF_PARAMS TESTS
% -----------------------------------------------------------------------
fprintf('\n=== EKF_PARAMS TESTS ===\n');

% T08
ok = isequal(size(ep.Q),[13,13]) && all(diag(ep.Q) > 0);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T08: Q is 13x13 positive diagonal\n');

% T09
ok = isequal(size(ep.R_gps),[4,4]) && all(diag(ep.R_gps) > 0);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T09: R_gps is 4x4 positive diagonal\n');

% -----------------------------------------------------------------------
%% EKF_INIT TEST
% -----------------------------------------------------------------------
fprintf('\n=== EKF_INIT TEST ===\n');

% T10
[xh0, P0] = ekf_init(x13, ep);
ok = isequal(size(xh0),[13,1]) && abs(norm(xh0(7:10))-1) < 1e-12;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T10: ekf_init — x_hat is 13x1, quaternion unit (norm=%.10f)\n', norm(xh0(7:10)));

% -----------------------------------------------------------------------
%% EKF_PROPAGATE TESTS
% -----------------------------------------------------------------------
fprintf('\n=== EKF_PROPAGATE TESTS ===\n');

x_h = xh0;  P_h = P0;
[x_h, P_h] = ekf_propagate(x_h, P_h, u_imu(1:3), u_imu(4:6), ep, params, 0.01);

% T11
ok = isequal(size(x_h),[13,1]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T11: propagate output is 13x1\n');

% T12
ok = abs(norm(x_h(7:10)) - 1) < 1e-12;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T12: quaternion normalized after propagation (norm=%.12f)\n', norm(x_h(7:10)));

% T13: P symmetric and positive definite
ok = norm(P_h - P_h','fro') < 1e-10 && min(eig(P_h)) > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T13: P is symmetric and PD (min_eig=%.2e)\n', min(eig(P_h)));

% -----------------------------------------------------------------------
%% EKF_UPDATE TESTS
% -----------------------------------------------------------------------
fprintf('\n=== EKF_UPDATE TESTS ===\n');

% T14: baro update pulls altitude estimate toward measurement
x_h14 = x_h;   P_h14 = P_h;
x_h14(3) = -95;   % estimated pd = -95m (thinks it's at 95m)
z_baro = 100;      % true measurement says 100m
[h_pred14, H14] = meas_baro(x_h14);
[x_h14_new, ~, innov14] = ekf_update(x_h14, P_h14, H14, ep.R_baro, z_baro, h_pred14);
ok = x_h14_new(3) < x_h14(3);  % pd should decrease (h increases toward 100m)
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T14: baro update pulls h toward measurement (innov=%.3f m)\n', innov14);

% T15: P decreases (becomes more certain) after measurement
trace_before = trace(P_h);
[~, P_h15] = ekf_update(x_h, P_h, H14, ep.R_baro, z_baro, h_pred14);
ok = trace(P_h15) < trace_before;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T15: trace(P) decreases after update (%.4f → %.4f)\n', trace_before, trace(P_h15));

% T16: quaternion normalized after update
[x_h16, ~] = ekf_update(x_h, P_h, H14, ep.R_baro, z_baro, h_pred14);
ok = abs(norm(x_h16(7:10)) - 1) < 1e-12;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T16: quaternion normalized after ekf_update\n');

% -----------------------------------------------------------------------
%% MEASUREMENT MODEL TESTS
% -----------------------------------------------------------------------
fprintf('\n=== MEASUREMENT MODEL TESTS ===\n');

% T17: meas_baro
[hp17, H17] = meas_baro(x13);
ok = abs(hp17 - (-x13(3))) < 1e-12 && H17(3) == -1 && all(H17([1,2,4:end]) == 0);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T17: meas_baro: h_pred=%.2f m, H(3)=%.0f\n', hp17, H17(3));

% T18: meas_pitot
[hp18, H18] = meas_pitot(x13);
Va18 = sqrt(x13(4)^2+x13(5)^2+x13(6)^2);
ok = abs(hp18-Va18)<1e-12 && abs(H18(4)-x13(4)/Va18)<1e-10;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T18: meas_pitot: Va_pred=%.3f m/s, H(4)=%.4f\n', hp18, H18(4));

% T19: meas_mag
[hp19, H19] = meas_mag(x13);
ok = isscalar(hp19) && any(H19(7:10) ~= 0);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T19: meas_mag: psi_pred=%.4f rad, H nonzero in q block\n', hp19);

% T20: meas_gps
[hp20, H20] = meas_gps(x13);
ok = isequal(size(hp20),[4,1]) && hp20(1)==x13(1) && hp20(2)==x13(2);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T20: meas_gps: h_pred(1:2)=[%.2f, %.2f] = [pn, pe]\n', hp20(1), hp20(2));

% T21: meas_gps row 3 Jacobian matches pitot (same Va formula)
[~, H18b] = meas_pitot(x13);
[~, H20b] = meas_gps(x13);
ok = norm(H20b(3,4:6) - H18b(4:6)) < 1e-10;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T21: GPS Vg Jacobian matches pitot Jacobian\n');

% -----------------------------------------------------------------------
%% FULL EKF TEST
% -----------------------------------------------------------------------
fprintf('\n=== FULL EKF TESTS ===\n');

sensor_state = sensors_init(sparams, x13);
[xhf, Pf] = ekf_init(x13, ep);

% Inject initial error
xhf(1) = xhf(1) + 5;  % 5m north position error

% T22: ekf output sizes
y_f = struct('gyro',u_imu(1:3),'accel',u_imu(4:6),...
             'baro',100,'baro_new',true, ...
             'pitot',25,'pitot_new',true, ...
             'mag',0,'mag_new',true, ...
             'gps',[0;0;25;0],'gps_new',true);
[xhf2, Pf2, ~] = ekf(xhf, Pf, y_f, ep, params, 0.01);
ok = isequal(size(xhf2),[13,1]) && isequal(size(Pf2),[13,13]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T22: ekf() output: x_hat 13x1, P 13x13\n');

% T23: quaternion stays normalized
ok = abs(norm(xhf2(7:10)) - 1) < 1e-12;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T23: quaternion normalized through full ekf step\n');

% T24: after many GPS updates, position error should decrease
x_run = x13;  xh_run = xhf;  Ph_run = Pf;
ss_run = sensors_init(sparams, x13);
for k = 1:500   % 5 seconds = 5 GPS updates
    [fm_r,~,~,~] = forces_moments(x_run, u_trim, zeros(3,1), zeros(3,1), params);
    [y_r, ss_run] = sensors(x_run, fm_r, ss_run, sparams, params, 0.01);
    [xh_run, Ph_run] = ekf(xh_run, Ph_run, y_r, ep, params, 0.01);
    x_run = mav_dynamics(x_run, fm_r, params, 0.01);
end
pos_err_final = norm(xh_run(1:3) - x_run(1:3));
ok = pos_err_final < 2.0;   % should be within 2m after 5s
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T24: position error after 5s = %.3f m (target <2m)\n', pos_err_final);

% -----------------------------------------------------------------------
%% SUMMARY
% -----------------------------------------------------------------------
total = pass+fail;
fprintf('\n========================================\n');
fprintf('  Results: %d / %d tests passed\n', pass, total);
if fail == 0
    fprintf('  ALL TESTS PASSED.\n');
else
    fprintf('  %d TEST(S) FAILED.\n', fail);
end
fprintf('========================================\n\n');
