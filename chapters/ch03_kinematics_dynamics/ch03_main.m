% =========================================================================
%  CH03_MAIN - Chapter 3: Kinematics and Dynamics
% =========================================================================
%  Simulates the 6-DOF equations of motion for the Aerosonde MAV.
%
%  State vector (13 states):
%    [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
%
%  This script covers:
%    Section 1 - Free-fall test (gravity only, zero initial velocity)
%                Verifies against analytical solution: altitude = 0.5*g*t^2
%    Section 2 - Level flight consistency (no moments → angular rates stay zero)
%    Section 3 - Quaternion norm conservation check
%    Plots      - Position, body velocities, Euler angles, angular rates
%
%  NOTE: Chapter 3 does NOT include aerodynamics yet.
%        Forces/moments are provided externally (gravity only here).
%        Aerodynamics will be added in Chapter 4.
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 3
% =========================================================================

% --- Auto-path setup (pwd-based, avoids Mac Editor temp-path bug) ---
atlas_root = fullfile(pwd, '..', '..');
addpath(atlas_root);
setup_paths(atlas_root);

clear; clc;
atlas_root = fullfile(pwd, '..', '..');
addpath(atlas_root);
setup_paths(atlas_root);

fprintf('\n========================================\n');
fprintf('  AtlasFC  --  Chapter 3: Dynamics\n');
fprintf('========================================\n\n');

% =========================================================================
%  PARAMETERS
% =========================================================================
params = mav_params();
g  = params.gravity;    % 9.81 m/s^2
m  = params.mass;       % 11 kg

dt    = 0.01;           % [s] time step
t_end = 10.0;           % [s] simulation duration
t_vec = 0 : dt : t_end;
N     = length(t_vec);

% =========================================================================
%  SECTION 1 — Free-fall test (gravity only)
% =========================================================================
fprintf('--- SECTION 1: Free-fall test (gravity only) ---\n');

% Initial state: level hover, everything zero
q0     = euler_to_quaternion(0, 0, 0);
state0 = [0; 0; 0;    % pn, pe, pd
          0; 0; 0;    % u, v, w
          q0;         % e0, e1, e2, e3
          0; 0; 0];   % p, q, r

% Storage
state_log = zeros(13, N);
state_log(:, 1) = state0;
state = state0;

for k = 2 : N
    % Rotate gravity from inertial (NED) to body frame
    R_vb       = quaternion_to_rotation(state(7:10));
    f_grav_body = R_vb * [0; 0; m * g];

    % No aerodynamics → moments = 0
    fm = [f_grav_body; 0; 0; 0];

    state = mav_dynamics(state, fm, params, dt);
    state_log(:, k) = state;
end

% Extract signals
pn    = state_log(1, :);
pe    = state_log(2, :);
pd    = state_log(3, :);    % positive = downward
u_vel = state_log(4, :);
v_vel = state_log(5, :);
w_vel = state_log(6, :);
p_l   = state_log(11, :);
q_l   = state_log(12, :);
r_l   = state_log(13, :);

% Convert quaternion history → Euler angles for plotting
phi_l   = zeros(1, N);
theta_l = zeros(1, N);
psi_l   = zeros(1, N);
for k = 1:N
    [phi_l(k), theta_l(k), psi_l(k)] = quaternion_to_euler(state_log(7:10, k));
end

% Analytical check: vehicle falls from rest → altitude = 0.5*g*t^2
pd_analytical = 0.5 * g * t_vec.^2;
pd_error = max(abs(pd - pd_analytical));
fprintf('  Max altitude error vs analytical: %.2e m\n', pd_error);
if pd_error < 1e-3
    fprintf('  [PASS] Free-fall matches analytical solution.\n\n');
else
    fprintf('  [FAIL] Free-fall error exceeds threshold!\n\n');
end

% =========================================================================
%  SECTION 2 — Zero moments → angular rates stay zero
% =========================================================================
fprintf('--- SECTION 2: No moments -> angular rates unchanged ---\n');
s2  = [0;0;0; 10;0;0; euler_to_quaternion(0,0,0); 0;0;0];
fm2 = zeros(6,1);
s2_new = mav_dynamics(s2, fm2, params, dt);
rate_change = norm(s2_new(11:13) - s2(11:13));
fprintf('  Angular rate change (1 step, no moments): %.2e rad/s\n', rate_change);
if rate_change < 1e-12
    fprintf('  [PASS] No spurious angular acceleration.\n\n');
else
    fprintf('  [FAIL] Unexpected angular rate change!\n\n');
end

% =========================================================================
%  SECTION 3 — Quaternion norm conservation
% =========================================================================
fprintf('--- SECTION 3: Quaternion norm conservation ---\n');
e0_l = state_log(7, :);
e1_l = state_log(8, :);
e2_l = state_log(9, :);
e3_l = state_log(10,:);
qnorm_drift = max(abs(sqrt(e0_l.^2 + e1_l.^2 + e2_l.^2 + e3_l.^2) - 1));
fprintf('  Max quaternion norm deviation over %.0f s: %.2e\n', t_end, qnorm_drift);
if qnorm_drift < 1e-10
    fprintf('  [PASS] Quaternion norm conserved.\n\n');
else
    fprintf('  [FAIL] Quaternion norm drifted!\n\n');
end

% =========================================================================
%  PLOTS
% =========================================================================

% --- Position ---
figure('Name', 'Ch03: Position (NED)', 'NumberTitle', 'off');
subplot(3,1,1); plot(t_vec, pn, 'b', 'LineWidth', 1.5);
ylabel('p_n [m]'); title('North'); grid on;

subplot(3,1,2); plot(t_vec, pe, 'g', 'LineWidth', 1.5);
ylabel('p_e [m]'); title('East'); grid on;

subplot(3,1,3);
plot(t_vec, pd, 'r', 'LineWidth', 1.5); hold on;
plot(t_vec, pd_analytical, 'k--', 'LineWidth', 1, 'DisplayName', 'Analytical 0.5gt^2');
ylabel('p_d [m]'); title('Down (positive = falling)');
xlabel('Time [s]'); legend('Simulated','Analytical'); grid on;

% --- Body velocities ---
figure('Name', 'Ch03: Body-frame Velocities', 'NumberTitle', 'off');
subplot(3,1,1); plot(t_vec, u_vel, 'b', 'LineWidth', 1.5);
ylabel('u [m/s]'); title('Body-x (forward)'); grid on;

subplot(3,1,2); plot(t_vec, v_vel, 'g', 'LineWidth', 1.5);
ylabel('v [m/s]'); title('Body-y (right)'); grid on;

subplot(3,1,3); plot(t_vec, w_vel, 'r', 'LineWidth', 1.5);
ylabel('w [m/s]'); title('Body-z (down)'); xlabel('Time [s]'); grid on;

% --- Euler angles ---
figure('Name', 'Ch03: Euler Angles', 'NumberTitle', 'off');
subplot(3,1,1); plot(t_vec, rad2deg(phi_l), 'b', 'LineWidth', 1.5);
ylabel('\phi [deg]'); title('Roll'); grid on;

subplot(3,1,2); plot(t_vec, rad2deg(theta_l), 'g', 'LineWidth', 1.5);
ylabel('\theta [deg]'); title('Pitch'); grid on;

subplot(3,1,3); plot(t_vec, rad2deg(psi_l), 'r', 'LineWidth', 1.5);
ylabel('\psi [deg]'); title('Yaw'); xlabel('Time [s]'); grid on;

% --- Angular rates ---
figure('Name', 'Ch03: Angular Rates', 'NumberTitle', 'off');
subplot(3,1,1); plot(t_vec, rad2deg(p_l), 'b', 'LineWidth', 1.5);
ylabel('p [deg/s]'); title('Roll rate'); grid on;

subplot(3,1,2); plot(t_vec, rad2deg(q_l), 'g', 'LineWidth', 1.5);
ylabel('q [deg/s]'); title('Pitch rate'); grid on;

subplot(3,1,3); plot(t_vec, rad2deg(r_l), 'r', 'LineWidth', 1.5);
ylabel('r [deg/s]'); title('Yaw rate'); xlabel('Time [s]'); grid on;

fprintf('========================================\n');
fprintf('  Chapter 3 complete. 4 plot windows.\n');
fprintf('========================================\n\n');
