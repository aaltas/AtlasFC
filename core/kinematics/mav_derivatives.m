% =========================================================================
%  MAV_DERIVATIVES - Right-hand side of the 13-state equations of motion
% =========================================================================
%  Computes the time derivative of the full 13-state MAV state vector
%  given forces, moments and current state. This is the function f(x, u)
%  in the continuous-time system:
%
%    xdot = f(state, fm, params)
%
%  Called by:
%    mav_dynamics.m    — four times per RK4 step
%    trim_residuals.m  — to evaluate acceleration residuals at trim
%
%  State vector (13 x 1):
%    state(1:3)   = [pn; pe; pd]          - NED position [m]
%    state(4:6)   = [u;  v;  w]           - body-frame velocity [m/s]
%    state(7:10)  = [e0; e1; e2; e3]      - quaternion attitude
%    state(11:13) = [p;  q;  r]           - body angular rates [rad/s]
%
%  Forces & Moments (6 x 1):
%    fm(1:3) = [fx; fy; fz]  - total force  in body frame [N]
%    fm(4:6) = [l;  m;  n]   - total moment in body frame [N*m]
%
%  Output xdot (13 x 1) — same layout as state:
%    xdot(1:3)   = [pn_dot; pe_dot; pd_dot]    - position rates [m/s]
%    xdot(4:6)   = [u_dot;  v_dot;  w_dot]     - velocity rates [m/s^2]
%    xdot(7:10)  = [e0_dot; e1_dot; e2_dot; e3_dot] - quaternion rates
%    xdot(11:13) = [p_dot;  q_dot;  r_dot]     - angular acceleration [rad/s^2]
%
%  Usage:
%    xdot = mav_derivatives(state, fm, params)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 3
% =========================================================================

function xdot = mav_derivatives(state, fm, params)

    % --- Unpack state ---
    u  = state(4);   v  = state(5);   w  = state(6);
    e0 = state(7);   e1 = state(8);   e2 = state(9);   e3 = state(10);
    p  = state(11);  q  = state(12);  r  = state(13);

    % --- Unpack forces and moments ---
    fx    = fm(1);  fy = fm(2);  fz = fm(3);
    l_mom = fm(4);  m_mom = fm(5);  n_mom = fm(6);

    % --- Unpack parameters ---
    mass = params.mass;
    G1   = params.Gamma1;  G2 = params.Gamma2;
    G3   = params.Gamma3;  G4 = params.Gamma4;
    G5   = params.Gamma5;  G6 = params.Gamma6;
    G7   = params.Gamma7;  G8 = params.Gamma8;
    Jy   = params.Jy;

    % -----------------------------------------------------------------------
    % 1. TRANSLATIONAL KINEMATICS
    %    [pn_dot; pe_dot; pd_dot] = R_b^v * [u; v; w]
    %    R_b^v = (R_v^b)^T  (body → inertial NED)
    % -----------------------------------------------------------------------
    R_bv    = quaternion_to_rotation([e0; e1; e2; e3])';
    pos_dot = R_bv * [u; v; w];

    % -----------------------------------------------------------------------
    % 2. TRANSLATIONAL DYNAMICS  (Newton's 2nd law in rotating body frame)
    %    u_dot = r*v - q*w + fx/m
    %    v_dot = p*w - r*u + fy/m
    %    w_dot = q*u - p*v + fz/m
    % -----------------------------------------------------------------------
    u_dot = r*v - q*w + fx / mass;
    v_dot = p*w - r*u + fy / mass;
    w_dot = q*u - p*v + fz / mass;

    % -----------------------------------------------------------------------
    % 3. QUATERNION KINEMATICS  (Beard & McLain Eq. 3.21)
    %    e_dot = 0.5 * Xi(e) * [p; q; r]
    % -----------------------------------------------------------------------
    quat_dot = quaternion_kinematics([e0; e1; e2; e3], [p; q; r]);

    % -----------------------------------------------------------------------
    % 4. ROTATIONAL DYNAMICS  (Euler's equations with Gamma constants)
    %    p_dot = Gamma1*p*q - Gamma2*q*r + Gamma3*l + Gamma4*n
    %    q_dot = Gamma5*p*r - Gamma6*(p^2 - r^2) + (1/Jy)*m
    %    r_dot = Gamma7*p*q - Gamma1*q*r + Gamma4*l + Gamma8*n
    % -----------------------------------------------------------------------
    p_dot = G1*p*q - G2*q*r + G3*l_mom + G4*n_mom;
    q_dot = G5*p*r - G6*(p^2 - r^2)    + m_mom / Jy;
    r_dot = G7*p*q - G1*q*r + G4*l_mom + G8*n_mom;

    % --- Pack derivative vector (same layout as state) ---
    xdot = [pos_dot;                            % 1:3  pn_dot, pe_dot, pd_dot
            u_dot;  v_dot;  w_dot;              % 4:6  translational dynamics
            quat_dot;                           % 7:10 quaternion kinematics
            p_dot;  q_dot;  r_dot];             % 11:13 rotational dynamics

end
