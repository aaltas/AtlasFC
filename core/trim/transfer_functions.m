% =========================================================================
%  TRANSFER_FUNCTIONS - Analytical transfer function coefficients
% =========================================================================
%  Computes the "a-coefficients" used in the simplified single-channel
%  transfer functions for autopilot design (Chapter 6).
%
%  These are derived analytically from the equations of motion by
%  linearizing each channel independently around trim.
%  Reference: Beard & McLain Ch. 5, Slides 22-36.
%
%  -------------------------------------------------------------------------
%  ROLL CHANNEL  (Slide 24)
%    phi_ddot = -a_phi1 * phi_dot + a_phi2 * delta_a
%    phi(s) = [a_phi2 / s(s + a_phi1)] * delta_a(s)
%
%  SIDESLIP CHANNEL  (Slide 28)
%    beta_dot = -a_beta1 * beta + a_beta2 * delta_r
%    beta(s) = [a_beta2 / (s + a_beta1)] * delta_r(s)
%
%  PITCH CHANNEL  (Slide 31)
%    theta_ddot = -a_theta1*theta_dot - a_theta2*theta + a_theta3*delta_e
%    theta(s) = [a_theta3 / (s^2 + a_theta1*s + a_theta2)] * delta_e(s)
%
%  AIRSPEED CHANNEL  (Slide 35-36)
%    Va_dot = -a_V1*Va + a_V2*delta_t - a_V3*theta
%    Va(s) = [a_V2/(s+a_V1)]*delta_t(s) - [a_V3/(s+a_V1)]*theta(s)
%
%  -------------------------------------------------------------------------
%  Inputs:
%    x_trim [12x1] - trim state  [pn,pe,pd,u,v,w,phi,theta,psi,p,q,r]
%    u_trim [4x1]  - trim inputs [delta_e, delta_a, delta_r, delta_t]
%    params        - struct from mav_params()
%
%  Output:
%    tf  - struct with all a-coefficients:
%          tf.a_phi1, tf.a_phi2
%          tf.a_beta1, tf.a_beta2
%          tf.a_theta1, tf.a_theta2, tf.a_theta3
%          tf.a_V1, tf.a_V2, tf.a_V3
%          tf.Va_star, tf.alpha_star, tf.theta_star, tf.delta_t_star
%
%  Usage:
%    tf = transfer_functions(x_trim, u_trim, params);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 5, Slides 22-36
% =========================================================================

function tf = transfer_functions(x_trim, u_trim, params)

    % --- Extract trim quantities ---
    u_b     = x_trim(4);
    w_b     = x_trim(6);
    theta_s = x_trim(8);   % trim pitch angle [rad]

    Va_s    = sqrt(u_b^2 + w_b^2);          % trim airspeed [m/s]
    alpha_s = atan2(w_b, u_b);               % trim AoA [rad]
    delta_t_s = u_trim(4);                   % trim throttle [-]
    delta_e_s = u_trim(1);                   % trim elevator [rad]

    % Shorthand for frequently used quantities
    rho = params.rho;
    S   = params.S_wing;
    b   = params.b;
    c   = params.c;
    m   = params.mass;
    Jy  = params.Jy;
    g   = params.gravity;

    % Combined lateral Cp-coefficients (Beard & McLain Eq. 4.13)
    C_pp    = params.Gamma3 * params.C_ell_p    + params.Gamma4 * params.C_n_p;
    C_pda   = params.Gamma3 * params.C_ell_delta_a + params.Gamma4 * params.C_n_delta_a;

    % -----------------------------------------------------------------------
    % ROLL TRANSFER FUNCTION  (Slide 22-24)
    %   phi_ddot = -a_phi1*phi_dot + a_phi2*delta_a
    %
    %   a_phi1 = -rho*Va^2*S*b^2*C_pp / (4*Va)  =  -rho*Va*S*b^2*C_pp/4
    %   a_phi2 =  rho*Va^2*S*b*C_pda / 2
    % -----------------------------------------------------------------------
    tf.a_phi1 = -0.5 * rho * Va_s^2 * S * b * C_pp * (b / (2*Va_s));
    tf.a_phi2 =  0.5 * rho * Va_s^2 * S * b * C_pda;

    % -----------------------------------------------------------------------
    % SIDESLIP TRANSFER FUNCTION  (Slide 26-28)
    %   beta_dot = -a_beta1*beta + a_beta2*delta_r
    %
    %   a_beta1 = -(rho*Va*S / (2*m)) * C_Ybeta
    %   a_beta2 =  (rho*Va*S / (2*m)) * C_Ydelta_r
    % -----------------------------------------------------------------------
    tf.a_beta1 = -(rho * Va_s * S / (2*m)) * params.C_Y_beta;
    tf.a_beta2 =  (rho * Va_s * S / (2*m)) * params.C_Y_delta_r;

    % -----------------------------------------------------------------------
    % PITCH TRANSFER FUNCTION  (Slide 29-31)
    %   theta_ddot = -a_theta1*theta_dot - a_theta2*theta + a_theta3*delta_e
    %
    %   a_theta1 = -(rho*Va^2*S*c / (2*Jy)) * C_mq * (c/(2*Va))
    %            = -(rho*Va*S*c^2 / (4*Jy)) * C_mq
    %   a_theta2 = -(rho*Va^2*S*c / (2*Jy)) * C_m_alpha
    %   a_theta3 =  (rho*Va^2*S*c / (2*Jy)) * C_m_delta_e
    % -----------------------------------------------------------------------
    qbar_Sc_2Jy = rho * Va_s^2 * S * c / (2 * Jy);   % common factor

    tf.a_theta1 = -qbar_Sc_2Jy * params.C_m_q * (c / (2*Va_s));
    tf.a_theta2 = -qbar_Sc_2Jy * params.C_m_alpha;
    tf.a_theta3 =  qbar_Sc_2Jy * params.C_m_delta_e;

    % -----------------------------------------------------------------------
    % AIRSPEED TRANSFER FUNCTION  (Slide 34-36)
    %   Va_dot = -a_V1*Va + a_V2*delta_t - a_V3*theta
    %
    %   a_V1 = (rho*Va*S/m)*[C_D0 + C_Dalpha*alpha* + C_Ddelta_e*delta_e*]
    %          - (1/m)*dTp/dVa
    %   a_V2 = (1/m)*dTp/d(delta_t)
    %   a_V3 = g * cos(theta* - alpha*)
    %
    %   Thrust partials computed via finite differences on propulsion model.
    % -----------------------------------------------------------------------
    % Thrust partial derivatives (numerical, central difference)
    eps_Va = 0.5;      % m/s perturbation
    eps_dt = 0.01;     % throttle perturbation

    [fp_Va, ~, ~, ~] = forces_moments( ...
        build_x13(x_trim, alpha_s, Va_s + eps_Va), ...
        u_trim, zeros(3,1), zeros(3,1), params);
    [fm_Va, ~, ~, ~] = forces_moments( ...
        build_x13(x_trim, alpha_s, Va_s - eps_Va), ...
        u_trim, zeros(3,1), zeros(3,1), params);
    dTp_dVa = (fp_Va(1) - fm_Va(1)) / (2*eps_Va);

    u_plus  = u_trim;  u_plus(4)  = min(1, u_trim(4) + eps_dt);
    u_minus = u_trim;  u_minus(4) = max(0, u_trim(4) - eps_dt);
    [fp_dt, ~, ~, ~] = forces_moments( ...
        build_x13(x_trim, alpha_s, Va_s), u_plus,  zeros(3,1), zeros(3,1), params);
    [fm_dt, ~, ~, ~] = forces_moments( ...
        build_x13(x_trim, alpha_s, Va_s), u_minus, zeros(3,1), zeros(3,1), params);
    dTp_ddt = (fp_dt(1) - fm_dt(1)) / (u_plus(4) - u_minus(4));

    % C_D_alpha may not exist as a separate field — use numerical value
    % from aerodynamic polar: C_D ≈ C_D0 + (C_L0 + C_Lalpha*alpha)^2/(pi*e*AR)
    CL_trim = params.C_L_0 + params.C_L_alpha * alpha_s;
    CD_trim = params.C_D_0 + (params.C_L_0 + params.C_L_alpha * alpha_s)^2 ...
              / (pi * params.e * params.AR);
    CD_delta_e = 0;    % typically small, set to 0 (params.C_D_delta_e if available)
    if isfield(params, 'C_D_delta_e')
        CD_delta_e = params.C_D_delta_e;
    end

    tf.a_V1 = (rho * Va_s * S / m) * (params.C_D_0 + params.C_D_alpha * alpha_s ...
              + CD_delta_e * delta_e_s) - (1/m) * dTp_dVa;
    tf.a_V2 = (1/m) * dTp_ddt;
    tf.a_V3 = g * cos(theta_s - alpha_s);

    % -----------------------------------------------------------------------
    % STORE TRIM INFORMATION FOR REFERENCE
    % -----------------------------------------------------------------------
    tf.Va_star      = Va_s;
    tf.alpha_star   = alpha_s;
    tf.theta_star   = theta_s;
    tf.delta_t_star = delta_t_s;
    tf.delta_e_star = delta_e_s;

end

% =========================================================================
%  BUILD_X13 - Helper: rebuild 13-state vector at a given Va
%  (used for finite-difference thrust derivatives)
% =========================================================================
function x13 = build_x13(x12, alpha, Va)
    phi   = x12(7);
    theta = x12(8);
    psi   = x12(9);
    e     = euler_to_quaternion(phi, theta, psi);

    u_b = Va * cos(alpha);
    w_b = Va * sin(alpha);

    x13 = [x12(1); x12(2); x12(3);   % pn, pe, pd
           u_b; 0; w_b;               % u, v, w
           e(1); e(2); e(3); e(4);    % quaternion
           x12(10); x12(11); x12(12)];% p, q, r
end
