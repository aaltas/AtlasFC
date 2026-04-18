% =========================================================================
%  PROPULSION_FORCES_MOMENTS - Propeller thrust and motor torque
% =========================================================================
%  Computes propulsion force (thrust) and moment (torque) in body frame.
%
%  The propeller shaft is aligned with body-x (nose), so:
%    f_prop^b = [T_p; 0; 0]
%    m_prop^b = [Q_p; 0; 0]   (Q_p is negative: torque opposes rotation)
%
%  Algorithm (Beard & McLain Ch.4, Slides 35-37):
%
%  Step 1 — Motor input voltage:
%    V_in = V_max * delta_t        (delta_t in [0,1])
%
%  Step 2 — Solve for operating propeller speed Omega_op [rad/s]:
%    Quadratic: a*Omega^2 + b*Omega + c = 0
%    where:
%      a = (rho*D^5 / (2pi)^2) * C_Q0
%      b = (rho*D^4 / (2pi))   * C_Q1 * Va + KQ*KV/R
%      c = rho*D^3 * C_Q2 * Va^2 - (KQ/R)*V_in + KQ*i0
%    Take positive root: Omega_op = (-b + sqrt(b^2 - 4*a*c)) / (2*a)
%
%  Step 3 — Advance ratio:
%    J_op = 2*pi*Va / (Omega_op * D)
%
%  Step 4 — Aerodynamic coefficients:
%    C_T(J) = C_T2*J^2 + C_T1*J + C_T0
%    C_Q(J) = C_Q2*J^2 + C_Q1*J + C_Q0
%
%  Step 5 — Thrust and torque:
%    n   = Omega_op / (2*pi)        [rev/s]
%    T_p = rho * n^2 * D^4 * C_T   [N]
%    Q_p = rho * n^2 * D^5 * C_Q   [N*m]
%
%  Inputs:
%    Va      - airspeed [m/s]   (used for advance ratio and motor balance)
%    delta_t - throttle command [0, 1]
%    params  - struct from mav_params()
%
%  Outputs:
%    fp - [3x1] propulsion force  in body frame [N]    = [T_p; 0; 0]
%    mp - [3x1] propulsion moment in body frame [N*m]  = [-Q_p; 0; 0]
%
%  Usage:
%    [fp, mp] = propulsion_forces_moments(Va, delta_t, params)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 4, Slides 30-37
% =========================================================================

function [fp, mp] = propulsion_forces_moments(Va, delta_t, params)

    rho   = params.rho;
    D     = params.D_prop;
    KQ    = params.KQ;
    KV    = params.KV;
    R     = params.R_motor;
    i0    = params.i0;
    V_max = params.V_max;
    C_Q0  = params.C_Q0;
    C_Q1  = params.C_Q1;
    C_Q2  = params.C_Q2;
    C_T0  = params.C_T0;
    C_T1  = params.C_T1;
    C_T2  = params.C_T2;

    % --- Step 1: Motor input voltage ---
    delta_t = max(0, min(1, delta_t));    % clamp to [0,1]
    V_in    = V_max * delta_t;

    % --- Step 2: Solve quadratic for Omega_op ---
    % Quadratic: a*Omega^2 + b*Omega + c = 0
    a_coef = (rho * D^5 / (2*pi)^2) * C_Q0;
    b_coef = (rho * D^4 / (2*pi)) * C_Q1 * Va  +  KQ * KV / R;
    c_coef =  rho * D^3 * C_Q2 * Va^2  -  (KQ/R) * V_in  +  KQ * i0;

    % Discriminant — guard against numerical issues
    discriminant = b_coef^2 - 4 * a_coef * c_coef;
    if discriminant < 0
        discriminant = 0;   % prevent sqrt of negative
    end

    % Positive root only (physical propeller speed)
    Omega_op = (-b_coef + sqrt(discriminant)) / (2 * a_coef);
    Omega_op = max(Omega_op, 0);   % must be non-negative

    % --- Step 3: Advance ratio ---
    if Omega_op > 1e-3
        J_op = 2 * pi * Va / (Omega_op * D);
    else
        J_op = 0;
    end

    % --- Step 4: Propeller aerodynamic coefficients ---
    C_T = C_T2 * J_op^2 + C_T1 * J_op + C_T0;
    C_Q = C_Q2 * J_op^2 + C_Q1 * J_op + C_Q0;

    % --- Step 5: Thrust and torque ---
    n   = Omega_op / (2 * pi);         % [rev/s]
    T_p = rho * n^2 * D^4 * C_T;      % thrust [N]
    Q_p = rho * n^2 * D^5 * C_Q;      % torque [N*m]

    % Guard: thrust cannot be negative (propeller not designed for reverse)
    T_p = max(T_p, 0);

    % --- Assemble output ---
    % Thrust along body-x (+nose direction)
    % Torque about body-x (motor reaction torque opposes rotation → negative)
    fp = [T_p;  0; 0];
    mp = [-Q_p; 0; 0];

end
