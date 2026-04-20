% =========================================================================
%  CH05_MAIN - Chapter 5: Linear Design Models
% =========================================================================
%  Computes trim conditions, linearizes the nonlinear dynamics, extracts
%  decoupled longitudinal/lateral state-space models, and computes the
%  analytical transfer function coefficients needed for autopilot design.
%
%  Three test sections:
%    Section 1 - Trim at Va=25 m/s, level flight (gamma=0, R=inf)
%                Verify trim cost < 1e-6 and state makes physical sense
%    Section 2 - Linearization and decoupled models
%                Verify A matrix stability (eigenvalues), check B structure
%    Section 3 - Transfer function coefficients
%                Verify sign conventions and magnitudes (slides 22-36)
%
%  Architecture context (Slide 2):
%    This chapter sits between the "Unmanned Vehicle" block (Ch.3-4) and
%    the "Autopilot" block (Ch.6). The linearized models produced here
%    are the inputs to PID autopilot design.
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 5
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
fprintf('  AtlasFC  --  Chapter 5: Trim & Linearization\n');
fprintf('========================================\n\n');

params = mav_params();

% Desired flight condition
Va_star    = 25.0;   % m/s   - airspeed
gamma_star = 0.0;    % rad   - flight path angle (0 = level)
R_star     = inf;    % m     - turn radius (inf = straight)

% =========================================================================
%  SECTION 1 — Trim Calculation
% =========================================================================
fprintf('--- SECTION 1: Trim at Va=%.0f m/s, gamma=%.1f deg ---\n', ...
    Va_star, rad2deg(gamma_star));

[x_trim, u_trim, info] = compute_trim(Va_star, gamma_star, R_star, params);

fprintf('  Optimizer cost:  %.2e  ', info.cost);
if info.converged
    fprintf('[CONVERGED]\n');
else
    fprintf('[WARNING: cost above threshold]\n');
end

fprintf('  Residual breakdown:\n');
fprintf('    u_dot = %+.4f m/s^2  (thrust-drag balance)\n', info.u_dot);
fprintf('    w_dot = %+.4f m/s^2  (lift-weight balance)\n', info.w_dot);
fprintf('    q_dot = %+.4f rad/s^2 (pitch moment)\n',       info.q_dot);
fprintf('    p_dot = %+.4f rad/s^2 (roll moment / motor torque)\n', info.p_dot);

fprintf('\n  Trim state:\n');
fprintf('    Va      = %.4f m/s\n', info.Va);
fprintf('    alpha   = %.4f deg\n', rad2deg(info.alpha));
fprintf('    theta   = %.4f deg\n', rad2deg(info.theta));
fprintf('    phi     = %.4f deg\n', rad2deg(x_trim(7)));

fprintf('\n  Trim inputs:\n');
fprintf('    delta_e = %.4f rad  (%.2f deg)\n', u_trim(1), rad2deg(u_trim(1)));
fprintf('    delta_a = %.4f rad  (%.2f deg)\n', u_trim(2), rad2deg(u_trim(2)));
fprintf('    delta_r = %.4f rad\n', u_trim(3));
fprintf('    delta_t = %.4f  (%.1f%%)\n', u_trim(4), u_trim(4)*100);

% Sanity checks
alpha_deg = rad2deg(info.alpha);
if alpha_deg > 0 && alpha_deg < 15
    fprintf('\n  [PASS] Alpha in reasonable range (0-15 deg).\n');
else
    fprintf('\n  [WARN] Alpha = %.2f deg outside expected range.\n', alpha_deg);
end

if u_trim(4) > 0.1 && u_trim(4) < 0.95
    fprintf('  [PASS] Throttle in reasonable range.\n\n');
else
    fprintf('  [WARN] Throttle = %.2f outside expected range.\n\n', u_trim(4));
end

% =========================================================================
%  SECTION 2 — Linearization and Decoupled Models
% =========================================================================
fprintf('--- SECTION 2: Linearization (numerical Jacobians) ---\n');

[A, B] = linearize(x_trim, u_trim, params);

fprintf('  A matrix size: %dx%d  B matrix size: %dx%d\n\n', ...
    size(A,1), size(A,2), size(B,1), size(B,2));

% Decouple into longitudinal and lateral
[sys_lon, sys_lat] = decouple_models(A, B);

% Eigenvalues
eig_lon = eig(sys_lon.A);
eig_lat = eig(sys_lat.A);

fprintf('  Longitudinal eigenvalues (x_lon = [u,w,q,theta,h]):\n');
for i = 1:length(eig_lon)
    fprintf('    lambda_%d = %+.4f %+.4fi\n', i, real(eig_lon(i)), imag(eig_lon(i)));
end

fprintf('\n  Lateral eigenvalues (x_lat = [v,p,r,phi,psi]):\n');
for i = 1:length(eig_lat)
    fprintf('    lambda_%d = %+.4f %+.4fi\n', i, real(eig_lat(i)), imag(eig_lat(i)));
end

% Check: at least some eigenvalues should have negative real part (stable modes)
n_stable_lon = sum(real(eig_lon) < 0);
n_stable_lat = sum(real(eig_lat) < 0);
fprintf('\n  Stable longitudinal modes: %d/5\n', n_stable_lon);
fprintf('  Stable lateral modes:      %d/5\n', n_stable_lat);

if n_stable_lon >= 2
    fprintf('  [PASS] Longitudinal has stable modes.\n');
else
    fprintf('  [WARN] Longitudinal shows unexpected instability.\n');
end
if n_stable_lat >= 2
    fprintf('  [PASS] Lateral has stable modes.\n\n');
else
    fprintf('  [WARN] Lateral shows unexpected instability.\n\n');
end

% =========================================================================
%  SECTION 3 — Transfer Function Coefficients (Slides 22-36)
% =========================================================================
fprintf('--- SECTION 3: Analytical Transfer Function Coefficients ---\n');

tf = transfer_functions(x_trim, u_trim, params);

fprintf('\n  ROLL  (phi/delta_a):  phi(s) = a_phi2 / [s(s+a_phi1)]\n');
fprintf('    a_phi1 = %+.4f  (roll damping)\n',  tf.a_phi1);
fprintf('    a_phi2 = %+.4f  (roll authority)\n', tf.a_phi2);
if tf.a_phi1 > 0 && tf.a_phi2 > 0
    fprintf('    [PASS] Signs correct (a_phi1>0, a_phi2>0).\n');
else
    fprintf('    [FAIL] Unexpected sign.\n');
end

fprintf('\n  SIDESLIP  (beta/delta_r):  beta(s) = a_beta2 / (s+a_beta1)\n');
fprintf('    a_beta1 = %+.4f  (sideslip damping)\n', tf.a_beta1);
fprintf('    a_beta2 = %+.4f  (rudder authority)\n', tf.a_beta2);
if tf.a_beta1 > 0
    fprintf('    [PASS] a_beta1>0 (stable sideslip mode).\n');
else
    fprintf('    [FAIL] a_beta1<=0 unexpected.\n');
end

fprintf('\n  PITCH  (theta/delta_e):  theta(s) = a_theta3 / (s^2+a_theta1*s+a_theta2)\n');
fprintf('    a_theta1 = %+.4f  (pitch rate damping)\n', tf.a_theta1);
fprintf('    a_theta2 = %+.4f  (pitch stiffness)\n',   tf.a_theta2);
fprintf('    a_theta3 = %+.4f  (elevator authority)\n', tf.a_theta3);
if tf.a_theta1 > 0 && tf.a_theta2 > 0
    fprintf('    [PASS] Pitch 2nd-order is stable (a_theta1>0, a_theta2>0).\n');
else
    fprintf('    [WARN] Pitch mode may be unstable.\n');
end

fprintf('\n  AIRSPEED  (Va/delta_t):  Va(s) = a_V2/(s+a_V1) * delta_t(s)\n');
fprintf('    a_V1 = %+.4f  (speed damping / drag slope)\n', tf.a_V1);
fprintf('    a_V2 = %+.4f  (throttle authority)\n',          tf.a_V2);
fprintf('    a_V3 = %+.4f  (pitch coupling to speed)\n',     tf.a_V3);
if tf.a_V1 > 0 && tf.a_V2 > 0
    fprintf('    [PASS] Airspeed channel signs correct.\n');
else
    fprintf('    [WARN] Unexpected airspeed channel sign.\n');
end

% =========================================================================
%  PLOTS
% =========================================================================

% --- Plot 1: Longitudinal eigenvalues (pole-zero map) ---
figure('Name', 'Ch05: Longitudinal Eigenvalues', 'NumberTitle', 'off');
plot(real(eig_lon), imag(eig_lon), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
hold on;
xline(0, 'k--', 'LineWidth', 1);
yline(0, 'k--', 'LineWidth', 1);
xlabel('Real'); ylabel('Imaginary');
title('Longitudinal Poles (x_{lon} = [u,w,q,\theta,h])');
grid on; axis equal;
legend('Poles', 'Location', 'best');

% --- Plot 2: Lateral eigenvalues ---
figure('Name', 'Ch05: Lateral Eigenvalues', 'NumberTitle', 'off');
plot(real(eig_lat), imag(eig_lat), 'bx', 'MarkerSize', 12, 'LineWidth', 2);
hold on;
xline(0, 'k--', 'LineWidth', 1);
yline(0, 'k--', 'LineWidth', 1);
xlabel('Real'); ylabel('Imaginary');
title('Lateral Poles (x_{lat} = [v,p,r,\phi,\psi])');
grid on; axis equal;
legend('Poles', 'Location', 'best');

% --- Plot 3: Step response of roll TF ---
% phi(s) = a_phi2 / (s^2 + a_phi1*s)  → impulse to delta_a
t_step = 0:0.01:5;
% Approximate step response: phi ~ (a_phi2/a_phi1)*(1 - exp(-a_phi1*t))
phi_step = (tf.a_phi2 / tf.a_phi1) * (1 - exp(-tf.a_phi1 * t_step));
figure('Name', 'Ch05: Roll Step Response (TF approx)', 'NumberTitle', 'off');
plot(t_step, rad2deg(phi_step), 'b', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\phi [deg]');
title(sprintf('Roll: approx step response to \\delta_a=1 rad\n(a_{\\phi1}=%.3f, a_{\\phi2}=%.3f)', ...
    tf.a_phi1, tf.a_phi2));
grid on;

% --- Plot 4: Pitch step response ---
% theta(s) = a_theta3 / (s^2 + a_theta1*s + a_theta2)
% Simulate via convolution (manual 2nd-order step response)
wn  = sqrt(tf.a_theta2);
zeta = tf.a_theta1 / (2*wn);
t_p = 0:0.01:8;
if zeta < 1
    wd = wn * sqrt(1 - zeta^2);
    theta_step = (tf.a_theta3 / tf.a_theta2) * ...
        (1 - exp(-zeta*wn*t_p) .* (cos(wd*t_p) + (zeta/sqrt(1-zeta^2))*sin(wd*t_p)));
else
    % overdamped
    r1 = -zeta*wn + wn*sqrt(zeta^2-1);
    r2 = -zeta*wn - wn*sqrt(zeta^2-1);
    theta_step = (tf.a_theta3 / tf.a_theta2) * ...
        (1 + (r2/(r1-r2))*exp(r1*t_p) - (r1/(r1-r2))*exp(r2*t_p));
end
figure('Name', 'Ch05: Pitch Step Response (TF)', 'NumberTitle', 'off');
plot(t_p, rad2deg(theta_step), 'g', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\theta [deg]');
title(sprintf('Pitch: step response to \\delta_e=1 rad (\\zeta=%.3f, \\omega_n=%.3f rad/s)', ...
    zeta, wn));
grid on;

fprintf('\n========================================\n');
fprintf('  Chapter 5 complete. 4 plot windows.\n');
fprintf('========================================\n\n');
fprintf('  Trim and linearization summary:\n');
fprintf('    Va*     = %.2f m/s\n',      tf.Va_star);
fprintf('    alpha*  = %.3f deg\n',      rad2deg(tf.alpha_star));
fprintf('    theta*  = %.3f deg\n',      rad2deg(tf.theta_star));
fprintf('    delta_t*= %.3f (%.1f%%)\n', tf.delta_t_star, tf.delta_t_star*100);
fprintf('    delta_e*= %.4f rad\n',      tf.delta_e_star);

% =========================================================================
%  SAVE TRIM DATA — used by Ch.6, Ch.7, Ch.8
% =========================================================================
%  Saved variables:
%    x_trim     [12x1]  trim state vector
%    u_trim     [4x1]   trim control inputs [de, da, dr, dt]
%    tf_s       struct  transfer function coefficients
%    gains      struct  PID autopilot gains (computed once here)
%    Va_star    [m/s]   trim airspeed
%    gamma_star [rad]   trim flight path angle
%    R_star     [m]     trim turn radius
% =========================================================================
tf_s  = tf;   % rename to conventional name used in Ch.6+
gains = autopilot_gains(tf_s, params);

% which('mav_params') returns absolute path of the function — reliable
% regardless of MATLAB's current working directory.
params_dir = fileparts(which('mav_params'));
trim_file  = fullfile(params_dir, 'trim_data.mat');
save(trim_file, 'x_trim', 'u_trim', 'tf_s', 'gains', ...
                'Va_star', 'gamma_star', 'R_star');
fprintf('\n  Trim data saved → %s\n', trim_file);
fprintf('  (Loaded automatically by Ch.6, Ch.7, Ch.8)\n\n');
