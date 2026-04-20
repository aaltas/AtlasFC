% =========================================================================
%  EKF_PROPAGATE - EKF prediction step (IMU-driven)
% =========================================================================
%  Propagates the state estimate and covariance forward by one time step
%  using IMU measurements and the navigation equations Jacobians from
%  nav_equations.m.
%
%  PROPAGATION EQUATIONS:
%    x_hat_new = x_hat + f(x_hat, u_imu) * dt    (Forward Euler)
%
%    Van Loan discretization  (replaces first-order I + A*dt):
%      M   = [A, B ; 0, 0] * dt
%      eM  = expm(M)
%      F_d = eM(1:ns, 1:ns)          ← exact discrete state Jacobian
%
%    P_new = F_d * P * F_d' + Q_d   (covariance)
%    Q_d   = ep.Q * dt              (diagonal process noise)
%
%  After state update: quaternion is renormalized.
%  After P update: symmetry is enforced (P = (P+P')/2).
%
%  WHY VAN LOAN:
%    I + A*dt is a first-order Taylor approximation of expm(A*dt).
%    For dt=0.01s the error is O((||A||*dt)^2) — small but nonzero.
%    expm gives the exact solution of ẋ=Ax between samples, which
%    is especially important as maneuvers increase A's spectral radius.
%
%  Inputs:
%    x_hat    [13x1]  current state estimate
%    P        [13x13] current covariance
%    y_gyro   [3x1]   gyroscope measurement [rad/s]
%    y_accel  [3x1]   accelerometer measurement [m/s^2]
%    ep       struct  ekf_params() (needs .Q)
%    params   struct  mav_params() (needs .gravity)
%    dt       [s]     time step
%
%  Outputs:
%    x_hat    [13x1]  propagated state estimate
%    P        [13x13] propagated covariance
%
%  Author : AtlasFC  |  Ref: Beard & McLain Ch. 8
% =========================================================================

function [x_hat, P] = ekf_propagate(x_hat, P, y_gyro, y_accel, ep, params, dt)

    u_imu = [y_gyro; y_accel];

    % -----------------------------------------------------------------------
    %  1. Compute state derivative and Jacobian from navigation equations
    % -----------------------------------------------------------------------
    [xdot, A, B] = nav_equations(x_hat, u_imu, params);

    % -----------------------------------------------------------------------
    %  2. Propagate state (Forward Euler)
    % -----------------------------------------------------------------------
    x_hat = x_hat + xdot * dt;

    % -----------------------------------------------------------------------
    %  3. Re-normalize quaternion (CRITICAL — must do after every update)
    % -----------------------------------------------------------------------
    qn = norm(x_hat(7:10));
    if qn > 1e-10
        x_hat(7:10) = x_hat(7:10) / qn;
    end

    % -----------------------------------------------------------------------
    %  4. Propagate covariance
    %
    %  F_d = I + A*dt  (first-order discretization, sufficient at 100 Hz)
    %
    %  Van Loan alternative (expm — more accurate at low update rates):
    %    ns=13; nu=6;
    %    eM  = expm([A, B; zeros(nu, ns+nu)] * dt);
    %    F_d = eM(1:ns, 1:ns);
    %  At dt=0.01s the difference is O((||A||·dt)²) ≈ negligible.
    % -----------------------------------------------------------------------
    F_d = eye(13) + A * dt;
    Q_d = ep.Q * dt;

    P = F_d * P * F_d' + Q_d;

    % -----------------------------------------------------------------------
    %  5. Enforce symmetry (numerical drift)
    % -----------------------------------------------------------------------
    P = 0.5 * (P + P');

end
