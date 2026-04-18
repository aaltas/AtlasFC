% =========================================================================
%  TEST_CH02_COORDINATE_FRAMES — Unit tests for core/coordinate_frames/
%  AtlasFC Project
% =========================================================================
%  Each test makes a numeric assertion. Failures print FAIL with the error.
%  Run from anywhere — paths are resolved automatically.
% =========================================================================

clear; clc;

% --- Auto-path setup (pwd-based, avoids Mac Editor temp-path bug) ---
atlas_root = fullfile(pwd, '..');
addpath(atlas_root);
setup_paths(atlas_root);

clear; clc;
atlas_root = fullfile(pwd, '..');
addpath(atlas_root);
setup_paths(atlas_root);
tol = 1e-10;   % numerical tolerance
passed = 0;
failed = 0;

fprintf('=== AtlasFC Unit Tests: Chapter 2 Coordinate Frames ===\n\n');

%% --- Helper ---
function assert_near(val, expected, tol, name)
    err = max(abs(val(:) - expected(:)));
    if err < tol
        fprintf('  PASS: %s  (err=%.2e)\n', name, err);
    else
        fprintf('  FAIL: %s  (err=%.2e, tol=%.2e)\n', name, err, tol);
    end
end

%% TEST 1: rot_x orthonormality
R = rot_x(pi/3);
assert_near(R*R', eye(3), tol, 'rot_x: R*R^T = I');
assert_near(det(R), 1, tol, 'rot_x: det=1');

%% TEST 2: rot_y orthonormality
R = rot_y(pi/5);
assert_near(R*R', eye(3), tol, 'rot_y: R*R^T = I');
assert_near(det(R), 1, tol, 'rot_y: det=1');

%% TEST 3: rot_z orthonormality
R = rot_z(pi/7);
assert_near(R*R', eye(3), tol, 'rot_z: R*R^T = I');
assert_near(det(R), 1, tol, 'rot_z: det=1');

%% TEST 4: euler_to_rotation — identity case
R = euler_to_rotation(0, 0, 0);
assert_near(R, eye(3), tol, 'euler_to_rotation: zero angles → identity');

%% TEST 5: euler_to_rotation — 3-2-1 composition
phi=0.3; theta=0.2; psi=1.1;
R = euler_to_rotation(phi, theta, psi);
R_manual = rot_x(phi) * rot_y(theta) * rot_z(psi);
assert_near(R, R_manual, tol, 'euler_to_rotation: matches rot_x*rot_y*rot_z');

%% TEST 6: euler_to_rotation orthonormality
assert_near(R*R', eye(3), tol, 'euler_to_rotation: orthonormal');
assert_near(det(R), 1, tol, 'euler_to_rotation: det=1');

%% TEST 7: rotation_to_euler round-trip
phi0=0.4; theta0=-0.1; psi0=2.5;
R = euler_to_rotation(phi0, theta0, psi0);
[phi_r, theta_r, psi_r] = rotation_to_euler(R);
assert_near([phi_r; theta_r; psi_r], [phi0; theta0; psi0], tol, ...
    'rotation_to_euler: round-trip');

%% TEST 8: euler_to_quaternion — unit norm
q = euler_to_quaternion(0.3, 0.1, 1.2);
assert_near(norm(q), 1, tol, 'euler_to_quaternion: unit norm');

%% TEST 9: quaternion_to_rotation matches euler_to_rotation
phi=0.5; theta=-0.2; psi=0.8;
q = euler_to_quaternion(phi, theta, psi);
R1 = euler_to_rotation(phi, theta, psi);
R2 = quaternion_to_rotation(q);
assert_near(R1, R2, tol, 'quaternion_to_rotation: matches euler_to_rotation');

%% TEST 10: quaternion round-trip
[phi_q, theta_q, psi_q] = quaternion_to_euler(q);
assert_near([phi_q;theta_q;psi_q], [phi;theta;psi], tol, ...
    'quaternion_to_euler: round-trip');

%% TEST 11: airspeed_alpha_beta — pure forward
[Va, alpha, beta] = airspeed_alpha_beta(25, 0, 0);
assert_near(Va, 25, tol, 'airspeed: Va pure forward');
assert_near(alpha, 0, tol, 'airspeed: alpha=0 pure forward');
assert_near(beta, 0, tol, 'airspeed: beta=0 pure forward');

%% TEST 12: airspeed_alpha_beta — known angles
Va0=30; a0=deg2rad(10); b0=deg2rad(5);
ur = Va0*cos(a0)*cos(b0);
vr = Va0*sin(b0);
wr = Va0*sin(a0)*cos(b0);
[Va_c, alpha_c, beta_c] = airspeed_alpha_beta(ur, vr, wr);
assert_near(Va_c, Va0, 1e-6, 'airspeed: Va from known angles');
assert_near(alpha_c, a0, 1e-6, 'airspeed: alpha from known angles');
assert_near(beta_c, b0, 1e-6, 'airspeed: beta from known angles');

%% TEST 13: wind_to_body — recover body airspeed
alpha=deg2rad(8); beta=deg2rad(3); Va=28;
R_wb = wind_to_body(alpha, beta);
Va_body = R_wb * [Va; 0; 0];
[Va_c, alpha_c, beta_c] = airspeed_alpha_beta(Va_body(1), Va_body(2), Va_body(3));
assert_near(Va_c, Va, 1e-6, 'wind_to_body: Va preserved');
assert_near(alpha_c, alpha, 1e-6, 'wind_to_body: alpha preserved');
assert_near(beta_c, beta, 1e-6, 'wind_to_body: beta preserved');

%% TEST 14: body_to_wind inverse of wind_to_body
alpha=0.15; beta=0.08;
assert_near(body_to_wind(alpha,beta), wind_to_body(alpha,beta)', tol, ...
    'body_to_wind = wind_to_body^T');

%% TEST 15: stability_to_body inverse
alpha = 0.12;
assert_near(body_to_stability(alpha), stability_to_body(alpha)', tol, ...
    'body_to_stability = stability_to_body^T');

fprintf('\n=== ALL TESTS COMPLETED ===\n');
