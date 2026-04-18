% =========================================================================
%  CH04_MAIN - Chapter 4: Forces and Moments
% =========================================================================
%  Full 6-DOF simulation with realistic aerodynamic forces and moments.
%  This chapter adds the missing physics from Chapter 3:
%    - Gravity (quaternion-based, exact)
%    - Aerodynamics (nonlinear C_L/C_D, longitudinal + lateral)
%    - Propulsion (motor + propeller quadratic model)
%    - Wind (steady-state in NED)
%
%  Simulation scenarios:
%    Section 1 - Trimmed level flight: Va=25 m/s, zero control inputs.
%                Gravity + aero should be near-balanced (loose trim).
%    Section 2 - Elevator step response: delta_e deflected, observe pitch.
%    Section 3 - Throttle step: delta_t ramp, observe acceleration.
%    Section 4 - Wind effect: 5 m/s headwind changes Va, alpha, beta.
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 4
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
fprintf('  AtlasFC  --  Chapter 4: Forces & Moments\n');
fprintf('========================================\n\n');

% =========================================================================
%  PARAMETERS & INITIAL STATE
% =========================================================================
params = mav_params();

dt    = 0.01;      % [s]
t_end = 15.0;      % [s]
t_vec = 0 : dt : t_end;
N     = length(t_vec);

% Initial state: level flight, Va=25 m/s, altitude=100 m
phi0 = 0;  theta0 = 0;  psi0 = 0;
q0   = euler_to_quaternion(phi0, theta0, psi0);
state0 = [0; 0; -100;       % pn, pe, pd (NED; -100 = 100 m altitude)
          25; 0; 0;          % u, v, w  (25 m/s forward)
          q0;                % quaternion
          0; 0; 0];          % p, q, r

% Steady wind (zero for most tests)
wind_i_zero = zeros(3,1);
wind_b_zero = zeros(3,1);

% =========================================================================
%  SECTION 1 — Trimmed level flight (all controls at zero)
% =========================================================================
fprintf('--- SECTION 1: Level flight, zero control inputs ---\n');

delta_trim = [0; 0; 0; 0.5];   % [delta_e, delta_a, delta_r, delta_t=50%]

state     = state0;
state_log = zeros(13, N);
fm_log    = zeros(6,  N);
Va_log    = zeros(1,  N);
alpha_log = zeros(1,  N);
beta_log  = zeros(1,  N);
state_log(:,1) = state;

[fm0, Va0, alpha0_val, beta0] = forces_moments(state, delta_trim, wind_i_zero, wind_b_zero, params);
fm_log(:,1)  = fm0;
Va_log(1)    = Va0;
alpha_log(1) = alpha0_val;
beta_log(1)  = beta0;

fprintf('  Initial Va=%.2f m/s  alpha=%.2f deg  beta=%.2f deg\n', ...
    Va0, rad2deg(alpha0_val), rad2deg(beta0));
fprintf('  Initial forces: fx=%.2f  fy=%.2f  fz=%.2f [N]\n', fm0(1), fm0(2), fm0(3));
fprintf('  Initial moments: l=%.3f  m=%.3f  n=%.3f [N*m]\n\n', fm0(4), fm0(5), fm0(6));

for k = 2 : N
    [fm, Va_k, alpha_k, beta_k] = forces_moments(state, delta_trim, wind_i_zero, wind_b_zero, params);
    state = mav_dynamics(state, fm, params, dt);
    state_log(:,k) = state;
    fm_log(:,k)    = fm;
    Va_log(k)      = Va_k;
    alpha_log(k)   = alpha_k;
    beta_log(k)    = beta_k;
end

% Extract signals
pn    = state_log(1,:);
pe    = state_log(2,:);
pd    = state_log(3,:);
u_v   = state_log(4,:);
w_v   = state_log(6,:);
p_l   = state_log(11,:);
q_l   = state_log(12,:);
r_l   = state_log(13,:);

% Convert quaternion → Euler
phi_l   = zeros(1,N);
theta_l = zeros(1,N);
psi_l   = zeros(1,N);
for k = 1:N
    [phi_l(k), theta_l(k), psi_l(k)] = quaternion_to_euler(state_log(7:10,k));
end

fprintf('  Final altitude: %.1f m  (started at 100 m)\n', -state_log(3,end));
fprintf('  Final Va: %.2f m/s\n', Va_log(end));
fprintf('  Final theta: %.2f deg\n\n', rad2deg(theta_l(end)));

% =========================================================================
%  SECTION 2 — Thrust sanity: delta_t=0 vs delta_t=1
% =========================================================================
fprintf('--- SECTION 2: Propulsion check ---\n');
s_test = state0;
[fm_t0, Va_t, ~, ~] = forces_moments(s_test, [0;0;0;0.0], wind_i_zero, wind_b_zero, params);
[fm_t1, ~,   ~, ~]  = forces_moments(s_test, [0;0;0;1.0], wind_i_zero, wind_b_zero, params);
fprintf('  Throttle 0%%: fx = %.2f N  (gravity + aero only)\n', fm_t0(1));
fprintf('  Throttle 100%%: fx = %.2f N  (+ full thrust)\n', fm_t1(1));
T_diff = fm_t1(1) - fm_t0(1);
fprintf('  Thrust contribution: %.2f N\n\n', T_diff);
if T_diff > 0
    fprintf('  [PASS] Thrust is positive and increases fx.\n\n');
else
    fprintf('  [FAIL] Unexpected thrust sign!\n\n');
end

% =========================================================================
%  SECTION 3 — Gravity check: level attitude, zero airspeed
% =========================================================================
fprintf('--- SECTION 3: Gravity force check ---\n');
s_grav = [0;0;0; 0;0;0; euler_to_quaternion(0,0,0); 0;0;0];
fg     = gravity_force(s_grav(7:10), params);
mg     = params.mass * params.gravity;
fprintf('  Expected fz_grav = %.2f N (= m*g)\n', mg);
fprintf('  Computed fz_grav = %.2f N\n', fg(3));
if abs(fg(3) - mg) < 0.01
    fprintf('  [PASS] Gravity correct at level attitude.\n\n');
else
    fprintf('  [FAIL] Gravity mismatch!\n\n');
end

% =========================================================================
%  PLOTS
% =========================================================================

figure('Name', 'Ch04: Altitude and Airspeed', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t_vec, -pd, 'b', 'LineWidth', 1.5);
ylabel('Altitude [m]'); title('Altitude (positive up)'); grid on;
subplot(2,1,2);
plot(t_vec, Va_log, 'r', 'LineWidth', 1.5);
ylabel('V_a [m/s]'); title('Airspeed'); xlabel('Time [s]'); grid on;

figure('Name', 'Ch04: Euler Angles', 'NumberTitle', 'off');
subplot(3,1,1); plot(t_vec, rad2deg(phi_l),   'b', 'LineWidth', 1.5); ylabel('\phi [deg]');   title('Roll');  grid on;
subplot(3,1,2); plot(t_vec, rad2deg(theta_l), 'g', 'LineWidth', 1.5); ylabel('\theta [deg]'); title('Pitch'); grid on;
subplot(3,1,3); plot(t_vec, rad2deg(psi_l),   'r', 'LineWidth', 1.5); ylabel('\psi [deg]');   title('Yaw');   xlabel('Time [s]'); grid on;

figure('Name', 'Ch04: Aero Angles', 'NumberTitle', 'off');
subplot(2,1,1); plot(t_vec, rad2deg(alpha_log), 'b', 'LineWidth', 1.5);
ylabel('\alpha [deg]'); title('Angle of Attack'); grid on;
subplot(2,1,2); plot(t_vec, rad2deg(beta_log),  'r', 'LineWidth', 1.5);
ylabel('\beta [deg]'); title('Sideslip Angle'); xlabel('Time [s]'); grid on;

figure('Name', 'Ch04: Forces', 'NumberTitle', 'off');
subplot(3,1,1); plot(t_vec, fm_log(1,:), 'b', 'LineWidth', 1.5); ylabel('f_x [N]'); title('Body-x Force'); grid on;
subplot(3,1,2); plot(t_vec, fm_log(2,:), 'g', 'LineWidth', 1.5); ylabel('f_y [N]'); title('Body-y Force'); grid on;
subplot(3,1,3); plot(t_vec, fm_log(3,:), 'r', 'LineWidth', 1.5); ylabel('f_z [N]'); title('Body-z Force'); xlabel('Time [s]'); grid on;

fprintf('========================================\n');
fprintf('  Chapter 4 complete. 4 plot windows.\n');
fprintf('========================================\n\n');
