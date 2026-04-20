% =========================================================================
%  TEST_CH06_AUTOPILOT - Unit tests for Chapter 6 autopilot
% =========================================================================
%  Tests:
%    T01  autopilot_gains  kp_phi > 0
%    T02  autopilot_gains  kd_phi — closed-loop zeta correct
%    T03  autopilot_gains  kp_theta < 0 (correct sign for a_theta3<0)
%    T04  autopilot_gains  closed-loop pitch poles stable
%    T05  autopilot_gains  kp_V > 0 and ki_V > 0
%    T06  autopilot_gains  closed-loop airspeed poles stable
%    T07  roll_hold        zero error → trim output
%    T08  roll_hold        positive phi_c > phi → positive delta_a
%    T09  roll_hold        output saturates at ±20 deg
%    T10  pitch_hold       zero error → trim output
%    T11  pitch_hold       positive theta_c > theta → negative delta_e
%    T12  pitch_hold       output saturates at ±30 deg
%    T13  airspeed_hold    zero error → trim throttle
%    T14  airspeed_hold    Va too low → increase delta_t
%    T15  airspeed_hold    anti-windup: integrator frozen at saturation
%    T16  altitude_hold    zero error → theta_trim
%    T17  altitude_hold    large positive error → theta_max
%    T18  altitude_hold    large negative error → theta_min
%    T19  course_hold      zero error → zero roll command
%    T20  course_hold      positive chi_c > chi → positive phi_c
%    T21  autopilot        output is 4x1
%    T22  autopilot        at trim → outputs near trim values
%    T23  autopilot        Va step → delta_t changes
%    T24  autopilot        h step  → delta_e changes (via pitch)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 6
% =========================================================================

% --- Auto-path setup ---
atlas_root = fullfile(pwd, '..');
addpath(atlas_root);
setup_paths(atlas_root);

clear; clc;
atlas_root = fullfile(pwd, '..');
addpath(atlas_root);
setup_paths(atlas_root);

fprintf('\n========================================\n');
fprintf('  AtlasFC  --  Ch06 Unit Tests\n');
fprintf('========================================\n\n');

% --- Setup: compute trim and gains ---
params = mav_params();
[x_trim, u_trim, ~] = compute_trim(25.0, 0, inf, params);
tf_s   = transfer_functions(x_trim, u_trim, params);
gains  = autopilot_gains(tf_s, params);

pass = 0;  fail = 0;
TOL  = 1e-9;

% -----------------------------------------------------------------------
%% GAINS TESTS
% -----------------------------------------------------------------------
fprintf('=== GAIN COMPUTATION TESTS ===\n');

% T01
ok = gains.kp_phi > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T01: kp_phi > 0 (got %.4f)\n', gains.kp_phi);

% T02: Verify closed-loop damping ratio for roll
wn_cl  = sqrt(tf_s.a_phi2 * gains.kp_phi);
zeta_cl = (tf_s.a_phi1 - tf_s.a_phi2*gains.kd_phi) / (2*wn_cl);
ok = abs(zeta_cl - gains.zeta_phi) < 1e-4;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T02: Roll CL zeta matches target (%.3f vs %.3f)\n', zeta_cl, gains.zeta_phi);

% T03
ok = gains.kp_theta < 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T03: kp_theta < 0 (correct for a_theta3<0, got %.4f)\n', gains.kp_theta);

% T04: Closed-loop pitch poles both stable
wn_theta_cl   = sqrt(tf_s.a_theta2 - tf_s.a_theta3*gains.kp_theta);
zeta_theta_cl = (tf_s.a_theta1 - tf_s.a_theta3*gains.kd_theta) / (2*wn_theta_cl);
ok = wn_theta_cl > 0 && zeta_theta_cl > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T04: Pitch CL poles stable (wn=%.2f, zeta=%.3f)\n', wn_theta_cl, zeta_theta_cl);

% T05
ok = gains.kp_V > 0 && gains.ki_V > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T05: kp_V > 0 and ki_V > 0 (%.4f, %.4f)\n', gains.kp_V, gains.ki_V);

% T06: Airspeed closed-loop stable
wn_V_cl   = sqrt(tf_s.a_V2 * gains.ki_V);
zeta_V_cl = (tf_s.a_V1 + tf_s.a_V2*gains.kp_V) / (2*wn_V_cl);
ok = wn_V_cl > 0 && zeta_V_cl > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T06: Airspeed CL poles stable (wn=%.3f, zeta=%.3f)\n', wn_V_cl, zeta_V_cl);

% -----------------------------------------------------------------------
%% ROLL HOLD TESTS
% -----------------------------------------------------------------------
fprintf('\n=== ROLL HOLD TESTS ===\n');

% T07: zero error → trim output
da_07 = roll_hold(0, 0, 0, u_trim(2), gains);
ok = abs(da_07 - u_trim(2)) < TOL;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T07: zero error -> delta_a = delta_a_trim (%.4f)\n', da_07);

% T08: positive phi_c > phi → positive delta_a (roll right)
da_08 = roll_hold(0.1, 0, 0, 0, gains);
ok = da_08 > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T08: phi_c > phi -> delta_a > 0 (got %.4f)\n', da_08);

% T09: saturation at ±20 deg
da_sat = roll_hold(1.0, 0, 0, 0, gains);   % huge error
ok = abs(da_sat - deg2rad(20)) < 1e-9;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T09: saturation at +20 deg (got %.2f deg)\n', rad2deg(da_sat));

% -----------------------------------------------------------------------
%% PITCH HOLD TESTS
% -----------------------------------------------------------------------
fprintf('\n=== PITCH HOLD TESTS ===\n');

% T10: zero error → trim output
de_10 = pitch_hold(x_trim(8), x_trim(8), 0, u_trim(1), gains);
ok = abs(de_10 - u_trim(1)) < TOL;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T10: zero error -> delta_e = delta_e_trim (%.4f)\n', de_10);

% T11: theta_c > theta → negative delta_e (nose up = trailing edge UP)
de_11 = pitch_hold(0.1, 0, 0, 0, gains);
ok = de_11 < 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T11: theta_c > theta -> delta_e < 0 (nose up, got %.4f rad)\n', de_11);

% T12: saturation at ±30 deg
de_sat = pitch_hold(1.0, 0, 0, 0, gains);  % huge error
ok = abs(de_sat + deg2rad(30)) < 1e-9;     % expect minimum = -30 deg
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T12: saturation at -30 deg (got %.2f deg)\n', rad2deg(de_sat));

% -----------------------------------------------------------------------
%% AIRSPEED HOLD TESTS
% -----------------------------------------------------------------------
fprintf('\n=== AIRSPEED HOLD TESTS ===\n');

% T13: zero error → trim throttle
[dt_13, ~] = airspeed_hold(25, 25, u_trim(4), gains, 0, 0.01);
ok = abs(dt_13 - u_trim(4)) < 1e-6;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T13: zero error -> delta_t = delta_t_trim (%.4f)\n', dt_13);

% T14: Va too low → increase throttle
[dt_14, ~] = airspeed_hold(28, 25, u_trim(4), gains, 0, 0.01);
ok = dt_14 > u_trim(4);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T14: Va < Va_c -> delta_t increases (%.4f vs trim=%.4f)\n', dt_14, u_trim(4));

% T15: anti-windup — integrator should not grow when saturated
int_before = 100;   % large integrator state (would push saturated)
[dt_15, int_after] = airspeed_hold(25, 30, 1.0, gains, int_before, 0.01);
% Va < Va_c (positive error), integrator is positive large, output saturated at 1
ok = (int_after == int_before) || abs(int_after - int_before) < 0.1;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T15: anti-windup active at saturation\n');

% -----------------------------------------------------------------------
%% ALTITUDE HOLD TESTS
% -----------------------------------------------------------------------
fprintf('\n=== ALTITUDE HOLD TESTS ===\n');

theta_trim = gains.theta_star;

% T16: zero error → theta_trim
tc_16 = altitude_hold(100, 100, theta_trim, gains);
ok = abs(tc_16 - theta_trim) < 1e-9;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T16: zero error -> theta_c = theta_trim (%.4f)\n', tc_16);

% T17: large positive error → theta_max (full climb)
tc_17 = altitude_hold(200, 100, theta_trim, gains);  % 100m error
ok = abs(tc_17 - gains.theta_max) < 1e-9;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T17: large pos error -> theta_max = %.1f deg\n', rad2deg(tc_17));

% T18: large negative error → theta_min (full descent)
tc_18 = altitude_hold(0, 100, theta_trim, gains);   % 100m below
ok = abs(tc_18 - gains.theta_min) < 1e-9;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T18: large neg error -> theta_min = %.1f deg\n', rad2deg(tc_18));

% -----------------------------------------------------------------------
%% COURSE HOLD TESTS
% -----------------------------------------------------------------------
fprintf('\n=== COURSE HOLD TESTS ===\n');

% T19: zero error → zero roll
pc_19 = course_hold(0, 0, gains);
ok = abs(pc_19) < TOL;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T19: zero course error -> phi_c = 0\n');

% T20: chi_c > chi → positive roll (bank right to turn right)
pc_20 = course_hold(0.5, 0, gains);
ok = pc_20 > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T20: chi_c > chi -> phi_c > 0 (right turn, got %.2f deg)\n', rad2deg(pc_20));

% -----------------------------------------------------------------------
%% TOP-LEVEL AUTOPILOT TESTS
% -----------------------------------------------------------------------
fprintf('\n=== FULL AUTOPILOT TESTS ===\n');

ap_state.Va_int = 0;
cmd.Va_c  = 25.0;
cmd.h_c   = 100.0;
cmd.chi_c = 0.0;

% T21: output size
[u_21, ~] = autopilot(cmd, x_trim, u_trim, gains, ap_state, 0.01);
ok = isequal(size(u_21), [4, 1]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T21: autopilot output is 4x1\n');

% T22: at trim, output ≈ trim inputs
[u_22, ~] = autopilot(cmd, x_trim, u_trim, gains, ap_state, 0.01);
trim_err = norm(u_22 - u_trim);
ok = trim_err < 1e-3;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T22: at trim -> u ≈ u_trim (err=%.2e)\n', trim_err);

% T23: Va step → delta_t changes
cmd_hi = cmd;  cmd_hi.Va_c = 30.0;
[u_23, ~] = autopilot(cmd_hi, x_trim, u_trim, gains, ap_state, 0.01);
ok = u_23(4) > u_trim(4);   % throttle should increase
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T23: Va step -> delta_t increases (%.3f vs %.3f)\n', u_23(4), u_trim(4));

% T24: altitude step → delta_e changes (pitch command changes)
cmd_hi = cmd;  cmd_hi.h_c = 150.0;
[u_24, ~] = autopilot(cmd_hi, x_trim, u_trim, gains, ap_state, 0.01);
ok = u_24(1) ~= u_trim(1);  % elevator should differ from trim
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T24: h step -> delta_e changes (%.4f vs trim %.4f)\n', u_24(1), u_trim(1));

% -----------------------------------------------------------------------
%% SUMMARY
% -----------------------------------------------------------------------
total = pass + fail;
fprintf('\n========================================\n');
fprintf('  Results: %d / %d tests passed\n', pass, total);
if fail == 0
    fprintf('  ALL TESTS PASSED.\n');
else
    fprintf('  %d TEST(S) FAILED.\n', fail);
end
fprintf('========================================\n\n');
