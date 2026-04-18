% =========================================================================
%  AERODYNAMIC_FORCES_MOMENTS - Aerodynamic forces and moments in body frame
% =========================================================================
%  Computes aerodynamic forces [fx_a; fy_a; fz_a] and moments [l_a; m_a; n_a]
%  acting on the MAV body frame.
%
%  Two aerodynamic groups (Slide 11):
%    Longitudinal: fx, fz, pitching moment m_aero
%                  driven by alpha, q, delta_e
%    Lateral:      fy, rolling moment l, yawing moment n
%                  driven by beta, p, r, delta_a, delta_r
%
%  LONGITUDINAL (Slides 17, 22, 23):
%    F_lift = 0.5*rho*Va^2*S * [C_L(alpha) + C_Lq*(c/2Va)*q + C_Lde*delta_e]
%    F_drag = 0.5*rho*Va^2*S * [C_D(alpha) + C_Dq*(c/2Va)*q + C_Dde*|delta_e|]
%
%    Body-frame longitudinal forces (wind → body rotation by alpha):
%      [fx] = [cos(a)  -sin(a)] * [-F_drag]
%      [fz]   [sin(a)   cos(a)]   [-F_lift]
%
%    Pitching moment (Slide 23):
%      m_aero = 0.5*rho*Va^2*S*c * [C_m0 + C_malpha*alpha + C_mq*(c/2Va)*q + C_mde*delta_e]
%
%  LATERAL (Slide 24):
%    fy    = 0.5*rho*Va^2*S   * [C_Y0 + C_Ybeta*beta + C_Yp*(b/2Va)*p + C_Yr*(b/2Va)*r + C_Yda*delta_a + C_Ydr*delta_r]
%    l     = 0.5*rho*Va^2*S*b * [C_l0 + C_lbeta*beta + C_lp*(b/2Va)*p + C_lr*(b/2Va)*r + C_lda*delta_a + C_ldr*delta_r]
%    n_aero= 0.5*rho*Va^2*S*b * [C_n0 + C_nbeta*beta + C_np*(b/2Va)*p + C_nr*(b/2Va)*r + C_nda*delta_a + C_ndr*delta_r]
%
%  Inputs:
%    Va      - airspeed [m/s]
%    alpha   - angle of attack [rad]
%    beta    - sideslip angle [rad]
%    p       - roll  rate [rad/s]
%    q       - pitch rate [rad/s]
%    r       - yaw   rate [rad/s]
%    delta_e - elevator deflection [rad]
%    delta_a - aileron  deflection [rad]
%    delta_r - rudder   deflection [rad]
%    params  - struct from mav_params()
%
%  Outputs:
%    fa - [3x1] aerodynamic forces  in body frame [N]   [fx; fy; fz]
%    ma - [3x1] aerodynamic moments in body frame [N*m] [l;  m;  n ]
%
%  Usage:
%    [fa, ma] = aerodynamic_forces_moments(Va, alpha, beta, p, q, r, ...
%                                          delta_e, delta_a, delta_r, params)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 4, Slides 17-24
% =========================================================================

function [fa, ma] = aerodynamic_forces_moments(Va, alpha, beta, p, q, r, ...
                                                delta_e, delta_a, delta_r, params)

    % Guard against zero airspeed (on ground or at launch)
    if Va < 0.1
        fa = zeros(3,1);
        ma = zeros(3,1);
        return;
    end

    % --- Unpack frequently used parameters ---
    rho  = params.rho;
    S    = params.S_wing;
    b    = params.b;
    c    = params.c;

    % Dynamic pressure q_bar = 0.5*rho*Va^2
    q_bar = 0.5 * rho * Va^2;

    % Non-dimensional rate terms
    c_over_2Va = c / (2 * Va);
    b_over_2Va = b / (2 * Va);

    % -----------------------------------------------------------------------
    % 1. NONLINEAR LIFT AND DRAG COEFFICIENTS  (Slides 16-20)
    % -----------------------------------------------------------------------
    [CL_alpha, CD_alpha] = lift_drag_coefficients(alpha, params);

    % -----------------------------------------------------------------------
    % 2. LONGITUDINAL FORCES (Slides 17, 22)
    % -----------------------------------------------------------------------
    % Full lift and drag (with pitch rate q and elevator delta_e effects)
    F_lift = q_bar * S * (CL_alpha ...
             + params.C_L_q     * c_over_2Va * q ...
             + params.C_L_delta_e * delta_e);

    F_drag = q_bar * S * (CD_alpha ...
             + params.C_D_q     * c_over_2Va * q ...
             + params.C_D_delta_e * abs(delta_e));

    % Rotate from stability frame to body frame (rotation by alpha about body-y)
    % [fx]   [ cos(alpha)  -sin(alpha)] [-F_drag]
    % [fz] = [ sin(alpha)   cos(alpha)] [-F_lift]
    ca = cos(alpha);
    sa = sin(alpha);

    fx_aero =  ca * (-F_drag) - sa * (-F_lift);   % = -F_drag*ca + F_lift*sa
    fz_aero =  sa * (-F_drag) + ca * (-F_lift);   % = -F_drag*sa - F_lift*ca

    % -----------------------------------------------------------------------
    % 3. PITCHING MOMENT (Slide 23)
    % -----------------------------------------------------------------------
    m_aero = q_bar * S * c * (params.C_m_0       ...
             + params.C_m_alpha * alpha            ...
             + params.C_m_q     * c_over_2Va * q  ...
             + params.C_m_delta_e * delta_e);

    % -----------------------------------------------------------------------
    % 4. LATERAL FORCES AND MOMENTS (Slide 24)
    % -----------------------------------------------------------------------
    fy_aero = q_bar * S * (params.C_Y_0         ...
              + params.C_Y_beta    * beta         ...
              + params.C_Y_p      * b_over_2Va * p ...
              + params.C_Y_r      * b_over_2Va * r ...
              + params.C_Y_delta_a * delta_a       ...
              + params.C_Y_delta_r * delta_r);

    l_aero  = q_bar * S * b * (params.C_ell_0         ...
              + params.C_ell_beta    * beta             ...
              + params.C_ell_p      * b_over_2Va * p   ...
              + params.C_ell_r      * b_over_2Va * r   ...
              + params.C_ell_delta_a * delta_a          ...
              + params.C_ell_delta_r * delta_r);

    n_aero  = q_bar * S * b * (params.C_n_0         ...
              + params.C_n_beta    * beta             ...
              + params.C_n_p      * b_over_2Va * p   ...
              + params.C_n_r      * b_over_2Va * r   ...
              + params.C_n_delta_a * delta_a          ...
              + params.C_n_delta_r * delta_r);

    % -----------------------------------------------------------------------
    % 5. ASSEMBLE OUTPUT VECTORS
    % -----------------------------------------------------------------------
    fa = [fx_aero; fy_aero; fz_aero];   % [N]
    ma = [l_aero;  m_aero;  n_aero];    % [N*m]

end
