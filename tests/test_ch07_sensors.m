% =========================================================================
%  TEST_CH07_SENSORS - Unit tests for Chapter 7 sensor models
% =========================================================================
%  Tests:
%    T01  sensor_params   struct has gyro fields
%    T02  sensor_params   struct has accel fields
%    T03  sensor_params   struct has baro/pitot/mag fields
%    T04  sensor_params   struct has GPS fields and gps_hz
%    T05  sensors_init    all bias fields are zero
%    T06  sensors_init    gps_timer = 0 (fires immediately)
%    T07  gyro_model      output is 3x1
%    T08  gyro_model      zero noise sigma → output ≈ truth (no noise case)
%    T09  gyro_model      bias propagates (alpha < 1)
%    T10  accel_model     output is 3x1
%    T11  accel_model     zero noise → output ≈ specific force
%    T12  baro_model      output is scalar
%    T13  baro_model      zero noise → output ≈ true altitude
%    T14  pitot_model     output is scalar and non-negative
%    T15  pitot_model     zero noise → output ≈ true airspeed
%    T16  pitot_model     output clipped to >= 0
%    T17  mag_model       output is scalar
%    T18  mag_model       at zero roll/pitch → output ≈ psi (+ noise)
%    T19  gps_model       output is 4x1
%    T20  gps_model       fires at t=0 (gps_timer=0) → new measurement
%    T21  gps_model       between updates → holds last measurement
%    T22  sensors         output struct has all required fields
%    T23  sensors         gyro field is 3x1
%    T24  sensors         GPS timer decrements each call
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 7
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
fprintf('  AtlasFC  --  Ch07 Unit Tests\n');
fprintf('========================================\n\n');

% --- Setup ---
params  = mav_params();
sparams = sensor_params();

% Build a representative 13-state (trim at Va=25, h=100m, level)
[x_trim, u_trim, ~] = compute_trim(25.0, 0, inf, params);
e_q = euler_to_quaternion(x_trim(7), x_trim(8), x_trim(9));
x13 = [x_trim(1:6); e_q; x_trim(10:12)];
x13(3) = -100;   % h = 100 m

% Forces at trim
[fm, ~, ~, ~] = forces_moments(x13, u_trim, zeros(3,1), zeros(3,1), params);

% Sensor state
sensor_state = sensors_init(sparams, x13);

pass = 0;  fail = 0;

% -----------------------------------------------------------------------
%% SENSOR_PARAMS TESTS
% -----------------------------------------------------------------------
fprintf('=== SENSOR_PARAMS TESTS ===\n');

% T01
ok = isfield(sparams, 'sigma_gyro') && isfield(sparams, 'tau_gyro') && ...
     isfield(sparams, 'sigma_b_gyro');
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T01: sensor_params has gyro fields\n');

% T02
ok = isfield(sparams, 'sigma_accel') && isfield(sparams, 'tau_accel') && ...
     isfield(sparams, 'sigma_b_accel');
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T02: sensor_params has accel fields\n');

% T03
ok = isfield(sparams, 'sigma_baro') && isfield(sparams, 'sigma_pitot') && ...
     isfield(sparams, 'sigma_mag');
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T03: sensor_params has baro/pitot/mag fields\n');

% T04
ok = isfield(sparams, 'sigma_gps_pn') && isfield(sparams, 'sigma_gps_pe') && ...
     isfield(sparams, 'gps_hz') && sparams.gps_hz > 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T04: sensor_params has GPS fields and gps_hz=%.1f\n', sparams.gps_hz);

% -----------------------------------------------------------------------
%% SENSORS_INIT TESTS
% -----------------------------------------------------------------------
fprintf('\n=== SENSORS_INIT TESTS ===\n');

% T05
ss0 = sensors_init(sparams);
ok = all(ss0.gyro_bias == 0) && all(ss0.accel_bias == 0) && ...
     ss0.baro_bias == 0 && ss0.pitot_bias == 0 && all(ss0.gps_bias == 0);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T05: sensors_init — all biases zero\n');

% T06
ok = ss0.gps_timer == 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T06: sensors_init — gps_timer = 0 (fires immediately)\n');

% -----------------------------------------------------------------------
%% GYRO_MODEL TESTS
% -----------------------------------------------------------------------
fprintf('\n=== GYRO_MODEL TESTS ===\n');

% T07: output size
[y7, b7] = gyro_model(x13, zeros(3,1), sparams, 0.01);
ok = isequal(size(y7), [3,1]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T07: gyro_model output is 3x1\n');

% T08: zero noise sigma → output ≈ truth
sp_quiet = sparams;
sp_quiet.sigma_gyro  = 0;
sp_quiet.sigma_b_gyro = 0;
sp_quiet.tau_gyro    = 1e10;  % bias frozen at 0
rng(42);
[y8, ~] = gyro_model(x13, zeros(3,1), sp_quiet, 0.01);
truth8 = [x13(11); x13(12); x13(13)];
ok = norm(y8 - truth8) < 1e-10;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T08: zero noise → gyro ≈ truth (err=%.2e)\n', norm(y8 - truth8));

% T09: bias propagates (alpha < 1 so bias decays)
b_init = [0.1; 0.0; 0.0];
[~, b9] = gyro_model(x13, b_init, sp_quiet, 0.01);
alpha9 = exp(-0.01 / sparams.tau_gyro);
ok = abs(b9(1) - alpha9*b_init(1)) < 1e-12;  % sigma_b=0, so exact
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T09: bias propagation correct (alpha=%.6f)\n', alpha9);

% -----------------------------------------------------------------------
%% ACCEL_MODEL TESTS
% -----------------------------------------------------------------------
fprintf('\n=== ACCEL_MODEL TESTS ===\n');

% T10: output size
[y10, ~] = accel_model(x13, fm, zeros(3,1), sparams, params, 0.01);
ok = isequal(size(y10), [3,1]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T10: accel_model output is 3x1\n');

% T11: zero noise → output ≈ specific force
sp_quiet2 = sparams;
sp_quiet2.sigma_accel   = 0;
sp_quiet2.sigma_b_accel = 0;
sp_quiet2.tau_accel     = 1e10;
rng(1);
[y11, ~] = accel_model(x13, fm, zeros(3,1), sp_quiet2, params, 0.01);
R_bv = quaternion_to_rotation(x13(7:10));
f_true11 = fm(1:3)/params.mass - R_bv*[0;0;params.gravity];
ok = norm(y11 - f_true11) < 1e-10;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T11: zero noise → accel ≈ specific force (err=%.2e)\n', norm(y11-f_true11));

% -----------------------------------------------------------------------
%% BARO_MODEL TESTS
% -----------------------------------------------------------------------
fprintf('\n=== BARO_MODEL TESTS ===\n');

% T12: output is scalar
[y12, ~] = baro_model(x13, 0, sparams, 0.01);
ok = isscalar(y12);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T12: baro_model output is scalar (%.3f m)\n', y12);

% T13: zero noise → output ≈ true altitude
sp_bq = sparams;
sp_bq.sigma_baro   = 0;
sp_bq.sigma_b_baro = 0;
sp_bq.tau_baro     = 1e10;
rng(2);
[y13, ~] = baro_model(x13, 0, sp_bq, 0.01);
h_true = -x13(3);
ok = abs(y13 - h_true) < 1e-12;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T13: zero noise → baro ≈ true h (err=%.2e m)\n', abs(y13-h_true));

% -----------------------------------------------------------------------
%% PITOT_MODEL TESTS
% -----------------------------------------------------------------------
fprintf('\n=== PITOT_MODEL TESTS ===\n');

Va_test = sqrt(x13(4)^2 + x13(5)^2 + x13(6)^2);

% T14: output is scalar and non-negative
[y14, ~] = pitot_model(x13, Va_test, 0, sparams, 0.01);
ok = isscalar(y14) && y14 >= 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T14: pitot output is scalar and non-negative (%.3f m/s)\n', y14);

% T15: zero noise → output ≈ true Va
sp_pq = sparams;
sp_pq.sigma_pitot   = 0;
sp_pq.sigma_b_pitot = 0;
sp_pq.tau_pitot     = 1e10;
rng(3);
[y15, ~] = pitot_model(x13, Va_test, 0, sp_pq, 0.01);
ok = abs(y15 - Va_test) < 1e-12;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T15: zero noise → pitot ≈ Va (err=%.2e m/s)\n', abs(y15-Va_test));

% T16: output clipped to >= 0 even for large negative noise
sp_neg = sparams;
sp_neg.sigma_pitot = 0;
sp_neg.sigma_b_pitot = 0;
[y16, ~] = pitot_model(x13, 0, -100, sp_neg, 0.01);  % -100 m/s bias
ok = y16 >= 0;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T16: large negative bias → pitot clipped to >= 0 (got %.4f)\n', y16);

% -----------------------------------------------------------------------
%% MAG_MODEL TESTS
% -----------------------------------------------------------------------
fprintf('\n=== MAG_MODEL TESTS ===\n');

% T17: output is scalar
y17 = mag_model(x13, sparams);
ok = isscalar(y17);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T17: mag_model output is scalar (%.4f rad)\n', y17);

% T18: zero noise + level flight → output ≈ psi
sp_mq = sparams;
sp_mq.sigma_mag = 0;
rng(4);
y18 = mag_model(x13, sp_mq);
R_bv18 = quaternion_to_rotation(x13(7:10));
[~, ~, psi18] = rotation_to_euler(R_bv18);
err18 = abs(atan2(sin(y18-psi18), cos(y18-psi18)));
ok = err18 < 1e-10;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T18: zero noise → mag ≈ psi (err=%.2e rad)\n', err18);

% -----------------------------------------------------------------------
%% GPS_MODEL TESTS
% -----------------------------------------------------------------------
fprintf('\n=== GPS_MODEL TESTS ===\n');

last_gps0 = zeros(4,1);
gps_bias0 = zeros(4,1);

% T19: output is 4x1
[y19, gb19, gt19, lg19] = gps_model(x13, Va_test, gps_bias0, sparams, 0.01, 0, last_gps0);
ok = isequal(size(y19), [4,1]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T19: gps_model output is 4x1\n');

% T20: fires at gps_timer=0 → new measurement (timer resets to ~1s)
ok = gt19 > 0.9 / sparams.gps_hz;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T20: gps fired at timer=0 → timer reset to %.3f s\n', gt19);

% T21: between updates — holds last GPS measurement
gps_timer_mid = 0.5;  % halfway between updates
[y21a, gb21, gt21, lg21] = gps_model(x13, Va_test, gps_bias0, sparams, 0.01, ...
                                      gps_timer_mid, last_gps0);
% timer should decrement but GPS output should be last_gps0 (zeros)
ok = all(y21a == last_gps0) && gt21 < gps_timer_mid;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T21: GPS hold between updates (timer=%.3f → %.3f)\n', gps_timer_mid, gt21);

% -----------------------------------------------------------------------
%% TOP-LEVEL SENSORS TESTS
% -----------------------------------------------------------------------
fprintf('\n=== SENSORS (TOP-LEVEL) TESTS ===\n');

ss_test = sensors_init(sparams, x13);

% T22: output struct has all required fields
[y22, ss22] = sensors(x13, fm, ss_test, sparams, params, 0.01);
ok = isfield(y22,'gyro') && isfield(y22,'accel') && isfield(y22,'baro') && ...
     isfield(y22,'pitot') && isfield(y22,'mag') && isfield(y22,'gps');
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T22: sensors() output has all required fields\n');

% T23: gyro field is 3x1
ok = isequal(size(y22.gyro), [3,1]);
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T23: y.gyro is 3x1\n');

% T24: GPS timer decrements each call (when not firing)
ss_notfire = ss_test;
ss_notfire.gps_timer = 0.5;   % mid-interval, won't fire
[~, ss24] = sensors(x13, fm, ss_notfire, sparams, params, 0.01);
ok = abs(ss24.gps_timer - (0.5 - 0.01)) < 1e-10;
if ok; fprintf('  [PASS]'); pass=pass+1; else; fprintf('  [FAIL]'); fail=fail+1; end
fprintf(' T24: GPS timer decrements (%.3f → %.3f)\n', 0.5, ss24.gps_timer);

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
