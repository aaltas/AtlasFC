% =========================================================================
%  CH02_MAIN — Chapter 2: Coordinate Frames
%  AtlasFC Project | Beard & McLain "Small Unmanned Aircraft"
% =========================================================================
%  Topics covered:
%    1. Elementary rotation matrices (rot_x, rot_y, rot_z)
%    2. Euler angles to rotation matrix (DCM)
%    3. Rotation matrix back to Euler angles (round-trip test)
%    4. Euler <-> Quaternion conversions
%    5. Aerodynamic angles: airspeed, alpha, beta
%    6. Full coordinate frame transformation chain
%    7. 3D MAV visualization animation
%
%  Run from anywhere — paths are resolved automatically.
% =========================================================================

clear; close all; clc;

% --- Auto path setup: find AtlasFC root from this file's location ---
          % .../chapters/ch02_...
atlas_root = fullfile(pwd, '..', '..');            % AtlasFC root
setup_paths(atlas_root);

fprintf('=== AtlasFC | Chapter 2: Coordinate Frames ===\n\n');
%%
% -------------------------------------------------------------------------
%  SECTION 1: Elementary Rotation Matrices
% -------------------------------------------------------------------------
fprintf('--- 1. Elementary Rotation Matrices ---\n');

theta_test = pi/4;  % 45 degrees

Rx = rot_x(theta_test);
Ry = rot_y(theta_test);
Rz = rot_z(theta_test);

fprintf('rot_x(45°):\n'); disp(round(Rx, 4));
fprintf('rot_y(45°):\n'); disp(round(Ry, 4));
fprintf('rot_z(45°):\n'); disp(round(Rz, 4));

% Orthonormality check: R * R^T = I and det(R) = 1
fprintf('Orthonormality check (should all be ~1.0):\n');
fprintf('  det(Rx)=%.6f  det(Ry)=%.6f  det(Rz)=%.6f\n', ...
    det(Rx), det(Ry), det(Rz));
fprintf('  max|Rx*Rx^T - I|=%.2e\n', max(abs(Rx*Rx' - eye(3)), [], 'all'));

%%
% -------------------------------------------------------------------------
%  SECTION 2: Euler Angles to DCM (Direction Cosine Matrix)
% -------------------------------------------------------------------------
fprintf('\n--- 2. Euler to DCM ---\n');

phi   = deg2rad(15);   % roll  = 15 deg
theta = deg2rad(10);   % pitch = 10 deg
psi   = deg2rad(45);   % yaw   = 45 deg (heading Northeast)

R = euler_to_rotation(phi, theta, psi);
fprintf('R_v^b (inertial to body) for phi=15 deg, theta=10 deg, psi=45 deg:\n');
disp(round(R, 4));

% Transform gravity vector from NED to body frame
g_ned   = [0; 0; 9.81];        % gravity in NED frame [m/s^2]
g_body  = R * g_ned;           % gravity expressed in body frame
fprintf('Gravity in body frame: [%.4f, %.4f, %.4f] m/s^2\n', g_body);
fprintf('  x-body (forward): %.4f  (expected: -g*sin(theta)=%.4f)\n', ...
    g_body(1), -9.81*sin(theta));
fprintf('  y-body (right):   %.4f  (expected:  g*sin(phi)*cos(theta)=%.4f)\n', ...
    g_body(2), 9.81*sin(phi)*cos(theta));

%%
% -------------------------------------------------------------------------
%  SECTION 3: DCM back to Euler Angles (Round-trip test)
% -------------------------------------------------------------------------
fprintf('\n--- 3. DCM to Euler (round-trip) ---\n');

[phi_r, theta_r, psi_r] = rotation_to_euler(R);
fprintf('Original:   phi=%.4f, theta=%.4f, psi=%.4f [rad]\n', phi, theta, psi);
fprintf('Recovered:  phi=%.4f, theta=%.4f, psi=%.4f [rad]\n', phi_r, theta_r, psi_r);
fprintf('Error:      %.2e, %.2e, %.2e\n', phi-phi_r, theta-theta_r, psi-psi_r);

%%
% -------------------------------------------------------------------------
%  SECTION 4: Euler <-> Quaternion
% -------------------------------------------------------------------------
fprintf('\n--- 4. Euler <-> Quaternion ---\n');

q = euler_to_quaternion(phi, theta, psi);
fprintf('Quaternion q = [e0, e1, e2, e3]:\n');
fprintf('  [%.6f, %.6f, %.6f, %.6f]\n', q);
fprintf('  ||q|| = %.10f  (expected: 1.0)\n', norm(q));

% Compare quaternion-derived DCM against euler-derived DCM
R_q = quaternion_to_rotation(q);
fprintf('Max difference |R_euler - R_quat|: %.2e\n', max(abs(R(:)-R_q(:))));

% Quaternion -> Euler round-trip
[phi_q, theta_q, psi_q] = quaternion_to_euler(q);
fprintf('Quaternion->Euler error: %.2e, %.2e, %.2e\n', ...
    phi-phi_q, theta-theta_q, psi-psi_q);

%%
% -------------------------------------------------------------------------
%  SECTION 5: Airspeed, Angle of Attack, Sideslip
% -------------------------------------------------------------------------
fprintf('\n--- 5. Airspeed, Alpha, Beta ---\n');

% Trim flight: 25 m/s forward, slight angle of attack
u_r = 25;  v_r = 0;  w_r = 2;   % body-frame airspeed components [m/s]

[Va, alpha, beta] = airspeed_alpha_beta(u_r, v_r, w_r);
fprintf('Body-frame airspeed: [%.1f, %.1f, %.1f] m/s\n', u_r, v_r, w_r);
fprintf('Va    = %.4f m/s\n', Va);
fprintf('alpha = %.4f rad = %.2f deg\n', alpha, rad2deg(alpha));
fprintf('beta  = %.4f rad = %.2f deg\n', beta,  rad2deg(beta));

% Verify: wind frame [Va,0,0] rotated to body should recover original
R_wb    = wind_to_body(alpha, beta);
Va_body = R_wb * [Va; 0; 0];
fprintf('Va_body recovered from wind frame: [%.4f, %.4f, %.4f]\n', Va_body);
fprintf('  Error vs original:               [%.2e, %.2e, %.2e]\n', ...
    Va_body(1)-u_r, Va_body(2)-v_r, Va_body(3)-w_r);

%%
% -------------------------------------------------------------------------
%  SECTION 6: Full Frame Transformation Chain
% -------------------------------------------------------------------------
fprintf('\n--- 6. Full Frame Transformation Chain ---\n');
fprintf('Chain: Wind -> Stability -> Body -> Vehicle -> Inertial\n\n');

% Show gravity vector expressed in each frame
fprintf('Gravity vector in each frame:\n');
fprintf('  NED (inertial): [%.4f, %.4f, %.4f]\n', g_ned);
fprintf('  Body:           [%.4f, %.4f, %.4f]\n', R * g_ned);
fprintf('  Stability:      [%.4f, %.4f, %.4f]\n', body_to_stability(alpha) * R * g_ned);

%%
% -------------------------------------------------------------------------
%  SECTION 7: 3D MAV Visualization
% -------------------------------------------------------------------------
fprintf('\n--- 7. 3D MAV Animation ---\n');
fprintf('Five phases: north, east, pitch-up, roll, yaw sweep\n');

viewer = mav_viewer();
ts  = 0.05;    % time step [s]
T   = 20.0;    % total duration [s]
t_vec = 0 : ts : T;

for k = 1:length(t_vec)
    t    = t_vec(k);
    frac = t / T;

    if frac < 1/5
        % Phase 1: translate north
        pn = 5*(frac*5);  pe = 0;  h = 100;
        phi = 0;  theta = 0;  psi = 0;
    elseif frac < 2/5
        % Phase 2: translate east
        pn = 5;  pe = 5*((frac-0.2)*5);  h = 100;
        phi = 0;  theta = 0;  psi = pi/2;
    elseif frac < 3/5
        % Phase 3: pitch up
        pn = 5;  pe = 5;
        h     = 100 + 5*((frac-0.4)*5);
        phi   = 0;
        theta = deg2rad(15)*((frac-0.4)*5);
        psi   = pi/2;
    elseif frac < 4/5
        % Phase 4: roll (bank)
        pn = 5;  pe = 5;  h = 105;
        phi   = deg2rad(30)*((frac-0.6)*5);
        theta = 0;  psi = pi/2;
    else
        % Phase 5: yaw sweep (full 360)
        pn = 5;  pe = 5;  h = 105;
        phi = 0;  theta = 0;
        psi = pi/2 + 2*pi*((frac-0.8)*5);
    end

    viewer.update(pn, pe, h, phi, theta, psi);
    pause(ts);
end

fprintf('\nChapter 2 complete.\n');
fprintf('Core library: rot_x/y/z, euler_to_rotation, rotation_to_euler,\n');
fprintf('euler<->quaternion, airspeed_alpha_beta, wind/stability frames.\n');
