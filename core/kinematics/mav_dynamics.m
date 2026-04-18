% =========================================================================
%  MAV_DYNAMICS - 6-DOF Equations of Motion with RK4 Integration
% =========================================================================
%  Propagates the 13-state MAV state vector one time step forward using
%  Runge-Kutta 4th order (RK4) integration.
%
%  State vector (13 x 1):
%    state(1)  = pn   - North position [m]
%    state(2)  = pe   - East  position [m]
%    state(3)  = pd   - Down  position [m]  (positive = below ground)
%    state(4)  = u    - body-frame x velocity [m/s]
%    state(5)  = v    - body-frame y velocity [m/s]
%    state(6)  = w    - body-frame z velocity [m/s]
%    state(7)  = e0   - quaternion scalar
%    state(8)  = e1   - quaternion x
%    state(9)  = e2   - quaternion y
%    state(10) = e3   - quaternion z
%    state(11) = p    - roll  rate [rad/s]
%    state(12) = q    - pitch rate [rad/s]
%    state(13) = r    - yaw   rate [rad/s]
%
%  Forces & Moments input (6 x 1):
%    fm(1) = fx  - total force  in body x [N]
%    fm(2) = fy  - total force  in body y [N]
%    fm(3) = fz  - total force  in body z [N]
%    fm(4) = l   - rolling  moment [N*m]
%    fm(5) = m   - pitching moment [N*m]
%    fm(6) = n   - yawing   moment [N*m]
%
%  Inputs:
%    state  - [13x1] current state vector
%    fm     - [6x1]  forces and moments in body frame
%    params - struct from mav_params()
%    dt     - time step [s]
%
%  Outputs:
%    state_new - [13x1] propagated state vector
%
%  Usage:
%    state_new = mav_dynamics(state, fm, params, dt)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 3
% =========================================================================

function state_new = mav_dynamics(state, fm, params, dt)

    % --- RK4 Integration ---
    k1 = derivatives(state,           fm, params);
    k2 = derivatives(state + dt/2*k1, fm, params);
    k3 = derivatives(state + dt/2*k2, fm, params);
    k4 = derivatives(state + dt*k3,   fm, params);

    state_new = state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

    % --- Quaternion Renormalization ---
    % Numerical drift will slowly corrupt the unit-norm constraint; fix it.
    q_norm = norm(state_new(7:10));
    state_new(7:10) = state_new(7:10) / q_norm;

end

% =========================================================================
%  DERIVATIVES - Right-hand side of the 13-state EOM
% =========================================================================
function xdot = derivatives(state, fm, params)

    % --- Unpack state ---
    u  = state(4);
    v  = state(5);
    w  = state(6);
    % Quaternion
    e0 = state(7);
    e1 = state(8);
    e2 = state(9);
    e3 = state(10);
    % Angular rates
    p  = state(11);
    q  = state(12);
    r  = state(13);

    % --- Unpack forces and moments ---
    fx = fm(1);
    fy = fm(2);
    fz = fm(3);
    l  = fm(4);
    m  = fm(5);
    n  = fm(6);

    % --- Unpack parameters ---
    mass   = params.mass;
    G1     = params.Gamma1;
    G2     = params.Gamma2;
    G3     = params.Gamma3;
    G4     = params.Gamma4;
    G5     = params.Gamma5;
    G6     = params.Gamma6;
    G7     = params.Gamma7;
    G8     = params.Gamma8;
    Jy     = params.Jy;

    % -----------------------------------------------------------------------
    % 1. TRANSLATIONAL KINEMATICS
    %    [pn_dot; pe_dot; pd_dot] = R_b^v * [u; v; w]
    %    R_b^v = (R_v^b)^T  built from quaternion (Beard & McLain Eq. 2.16)
    % -----------------------------------------------------------------------
    R_bv = [(e1^2+e0^2-e2^2-e3^2),  2*(e1*e2-e0*e3),        2*(e1*e3+e0*e2);
             2*(e1*e2+e0*e3),        (e0^2-e1^2+e2^2-e3^2),  2*(e2*e3-e0*e1);
             2*(e1*e3-e0*e2),        2*(e2*e3+e0*e1),        (e0^2-e1^2-e2^2+e3^2)];

    pos_dot = R_bv * [u; v; w];

    pn_dot = pos_dot(1);
    pe_dot = pos_dot(2);
    pd_dot = pos_dot(3);

    % -----------------------------------------------------------------------
    % 2. TRANSLATIONAL DYNAMICS  (Newton's 2nd law in body frame)
    %    u_dot = r*v - q*w + fx/m
    %    v_dot = p*w - r*u + fy/m
    %    w_dot = q*u - p*v + fz/m
    % -----------------------------------------------------------------------
    u_dot = r*v - q*w + fx/mass;
    v_dot = p*w - r*u + fy/mass;
    w_dot = q*u - p*v + fz/mass;

    % -----------------------------------------------------------------------
    % 3. QUATERNION KINEMATICS  (Beard & McLain Eq. 3.21)
    %    e_dot = 0.5 * Xi(e) * [p; q; r]
    % -----------------------------------------------------------------------
    Xi = [-e1, -e2, -e3;
           e0, -e3,  e2;
           e3,  e0, -e1;
          -e2,  e1,  e0];

    quat_dot = 0.5 * Xi * [p; q; r];

    e0_dot = quat_dot(1);
    e1_dot = quat_dot(2);
    e2_dot = quat_dot(3);
    e3_dot = quat_dot(4);

    % -----------------------------------------------------------------------
    % 4. ROTATIONAL DYNAMICS  (Euler's equations with Gamma constants)
    %    p_dot = Gamma1*p*q - Gamma2*q*r + Gamma3*l + Gamma4*n
    %    q_dot = Gamma5*p*r - Gamma6*(p^2 - r^2) + (1/Jy)*m
    %    r_dot = Gamma7*p*q - Gamma1*q*r + Gamma4*l + Gamma8*n
    % -----------------------------------------------------------------------
    p_dot = G1*p*q - G2*q*r + G3*l + G4*n;
    q_dot = G5*p*r - G6*(p^2 - r^2) + (1/Jy)*m;
    r_dot = G7*p*q - G1*q*r + G4*l + G8*n;

    % --- Pack derivative vector ---
    xdot = [pn_dot; pe_dot; pd_dot;
            u_dot;  v_dot;  w_dot;
            e0_dot; e1_dot; e2_dot; e3_dot;
            p_dot;  q_dot;  r_dot];

end
