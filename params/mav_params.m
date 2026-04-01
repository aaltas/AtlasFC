% =========================================================================
%  MAV_PARAMS - Aircraft physical and aerodynamic parameters
% =========================================================================
%  This file contains all physical constants and aerodynamic coefficients
%  for the small UAV model used throughout the project. Values are based on
%  the Aerosonde UAV as described in Beard & McLain, Appendix E.
%
%  Usage:
%    params = mav_params();
%
%  The returned struct contains fields grouped by subsystem:
%    params.gravity, params.mass, params.Jx, ...
%    params.S_wing, params.b, params.c, ...
%    params.C_L_0, params.C_D_0, ...
%    params.C_prop, params.k_motor, ...
% =========================================================================

function params = mav_params()

    % --- Physical Constants ---
    params.gravity = 9.81;          % m/s^2

    % --- Mass and Inertia ---
    params.mass = 11.0;             % kg
    params.Jx   = 0.824;           % kg*m^2
    params.Jy   = 1.135;           % kg*m^2
    params.Jz   = 1.759;           % kg*m^2
    params.Jxz  = 0.120;           % kg*m^2

    % Gamma constants (precomputed for equations of motion)
    Gamma  = params.Jx * params.Jz - params.Jxz^2;
    params.Gamma  = Gamma;
    params.Gamma1 = (params.Jxz * (params.Jx - params.Jy + params.Jz)) / Gamma;
    params.Gamma2 = (params.Jz * (params.Jz - params.Jy) + params.Jxz^2) / Gamma;
    params.Gamma3 = params.Jz / Gamma;
    params.Gamma4 = params.Jxz / Gamma;
    params.Gamma5 = (params.Jz - params.Jx) / params.Jy;
    params.Gamma6 = params.Jxz / params.Jy;
    params.Gamma7 = ((params.Jx - params.Jy) * params.Jx + params.Jxz^2) / Gamma;
    params.Gamma8 = params.Jx / Gamma;

    % --- Wing Geometry ---
    params.S_wing = 0.55;          % m^2       - wing area
    params.b      = 2.90;          % m         - wingspan
    params.c      = 0.19;          % m         - mean chord
    params.e      = 0.9;           % -          - Oswald efficiency

    % --- Aerodynamic Coefficients (Longitudinal) ---
    params.C_L_0     =  0.23;
    params.C_L_alpha =  5.61;
    params.C_L_q     =  7.95;
    params.C_L_delta_e = 0.13;

    params.C_D_0     =  0.043;
    params.C_D_alpha =  0.030;
    params.C_D_q     =  0.0;
    params.C_D_delta_e = 0.0135;

    params.C_m_0     = -0.0135;
    params.C_m_alpha = -2.74;
    params.C_m_q     = -38.21;
    params.C_m_delta_e = -0.99;

    % --- Aerodynamic Coefficients (Lateral) ---
    params.C_Y_0       =  0.0;
    params.C_Y_beta    = -0.83;
    params.C_Y_p       =  0.0;
    params.C_Y_r       =  0.0;
    params.C_Y_delta_a =  0.075;
    params.C_Y_delta_r =  0.19;

    params.C_ell_0       =  0.0;
    params.C_ell_beta    = -0.13;
    params.C_ell_p       = -0.51;
    params.C_ell_r       =  0.25;
    params.C_ell_delta_a =  0.17;
    params.C_ell_delta_r =  0.0024;

    params.C_n_0       =  0.0;
    params.C_n_beta    =  0.073;
    params.C_n_p       = -0.069;
    params.C_n_r       = -0.095;
    params.C_n_delta_a = -0.011;
    params.C_n_delta_r = -0.069;

    % --- Propulsion ---
    params.C_prop  = 1.0;          % propeller efficiency coefficient
    params.k_motor = 80;           % motor constant
    params.S_prop  = 0.2027;       % m^2 - propeller disk area
    params.k_T_p   = 0.0;
    params.k_Omega = 0.0;

    % --- Initial Conditions ---
    params.pn0    = 0;     % m   - initial north position
    params.pe0    = 0;     % m   - initial east position
    params.pd0    = -100;  % m   - initial down position (NED, so negative = altitude)
    params.u0     = 25;    % m/s - initial body-x velocity
    params.v0     = 0;     % m/s - initial body-y velocity
    params.w0     = 0;     % m/s - initial body-z velocity
    params.phi0   = 0;     % rad - initial roll
    params.theta0 = 0;     % rad - initial pitch
    params.psi0   = 0;     % rad - initial yaw
    params.p0     = 0;     % rad/s - initial roll rate
    params.q0     = 0;     % rad/s - initial pitch rate
    params.r0     = 0;     % rad/s - initial yaw rate

end
