% =========================================================================
%  FORCES_MOMENTS - Total external forces and moments in body frame
% =========================================================================
%  Assembles all external forces and moments acting on the MAV:
%
%    f_total = f_gravity + f_aerodynamic + f_propulsion
%    m_total = m_aerodynamic + m_propulsion
%
%  Wind is accounted for by computing the relative airspeed:
%    V_air^b = V_body^b - V_wind^b
%  from which Va, alpha, beta are derived and fed to the aero model.
%
%  Inputs:
%    state   - [13x1] state vector:
%                [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
%    delta   - [4x1]  control inputs:
%                [delta_e, delta_a, delta_r, delta_t]
%                (elevator, aileron, rudder deflections [rad]; throttle [0,1])
%    wind_i  - [3x1]  steady-state wind in inertial (NED) frame [m/s]
%                     [w_n; w_e; w_d]   (set to zeros(3,1) for no wind)
%    wind_b  - [3x1]  gust wind in body frame [m/s]
%                     [u_wg; v_wg; w_wg] (set to zeros(3,1) for no gust)
%    params  - struct from mav_params()
%
%  Outputs:
%    fm  - [6x1] force and moment vector in body frame:
%                [fx; fy; fz; l; m; n]
%    Va  - airspeed [m/s]
%    alpha - angle of attack [rad]
%    beta  - sideslip angle  [rad]
%
%  Usage:
%    [fm, Va, alpha, beta] = forces_moments(state, delta, wind_i, wind_b, params)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 4
% =========================================================================

function [fm, Va, alpha, beta] = forces_moments(state, delta, wind_i, wind_b, params)

    % --- Unpack state ---
    u  = state(4);
    v  = state(5);
    w  = state(6);
    e0 = state(7);
    e1 = state(8);
    e2 = state(9);
    e3 = state(10);
    p  = state(11);
    q  = state(12);
    r  = state(13);

    % --- Unpack control inputs ---
    delta_e = delta(1);   % elevator [rad]
    delta_a = delta(2);   % aileron  [rad]
    delta_r = delta(3);   % rudder   [rad]
    delta_t = delta(4);   % throttle [0,1]

    % -----------------------------------------------------------------------
    % 1. WIND IN BODY FRAME  (Slide 42)
    %    V_wind^b = R_v^b * V_wind^i + V_gust^b
    % -----------------------------------------------------------------------
    % Rotation matrix R_v^b (inertial → body) from quaternion
    R_vb = [(e1^2+e0^2-e2^2-e3^2), 2*(e1*e2+e0*e3),       2*(e1*e3-e0*e2);
             2*(e1*e2-e0*e3),       (e0^2-e1^2+e2^2-e3^2), 2*(e2*e3+e0*e1);
             2*(e1*e3+e0*e2),       2*(e2*e3-e0*e1),        (e0^2-e1^2-e2^2+e3^2)];

    % Wind in body frame: steady (rotated from NED) + gust
    V_wind_body = R_vb * wind_i + wind_b;   % [3x1]

    % -----------------------------------------------------------------------
    % 2. RELATIVE AIRSPEED  (Slide 42)
    %    [u_r; v_r; w_r] = [u; v; w] - V_wind^b
    % -----------------------------------------------------------------------
    u_r = u - V_wind_body(1);
    v_r = v - V_wind_body(2);
    w_r = w - V_wind_body(3);

    % Airspeed, angle of attack, sideslip
    [Va, alpha, beta] = airspeed_alpha_beta(u_r, v_r, w_r);

    % -----------------------------------------------------------------------
    % 3. GRAVITY FORCE IN BODY FRAME  (Slide 5)
    % -----------------------------------------------------------------------
    fg = gravity_force([e0; e1; e2; e3], params);   % [3x1]

    % -----------------------------------------------------------------------
    % 4. AERODYNAMIC FORCES AND MOMENTS  (Slides 17-24)
    % -----------------------------------------------------------------------
    [fa, ma] = aerodynamic_forces_moments(Va, alpha, beta, p, q, r, ...
                                          delta_e, delta_a, delta_r, params);

    % -----------------------------------------------------------------------
    % 5. PROPULSION FORCES AND MOMENTS  (Slides 30-37)
    % -----------------------------------------------------------------------
    [fp, mp] = propulsion_forces_moments(Va, delta_t, params);

    % -----------------------------------------------------------------------
    % 6. ASSEMBLE TOTAL fm VECTOR
    % -----------------------------------------------------------------------
    f_total = fg + fa + fp;          % [3x1] total force  in body frame [N]
    m_total = ma + mp;               % [3x1] total moment in body frame [N*m]

    fm = [f_total; m_total];         % [6x1]

end
