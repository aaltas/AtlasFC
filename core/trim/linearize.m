% =========================================================================
%  LINEARIZE - Compute state-space matrices A and B via numerical Jacobians
% =========================================================================
%  Linearizes the nonlinear MAV dynamics around a trim point using
%  central finite differences.
%
%  System representation (12-state Euler angles):
%    x = [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]   (12 x 1)
%    u = [delta_e, delta_a, delta_r, delta_t]               (4  x 1)
%    xdot = A*(x - x_trim) + B*(u - u_trim)
%
%  WHY 12-STATE EULER (not 13-state quaternion)?
%    The quaternion 4-vector has a redundancy (norm = 1 constraint) that
%    makes the Jacobian rank-deficient in the quaternion direction.
%    Euler angles give a minimal 12-state representation suitable for
%    control design, avoiding this issue.
%
%  Numerical differentiation:
%    A(:,i) = [ f(x+eps*e_i, u) - f(x-eps*e_i, u) ] / (2*eps)
%    B(:,j) = [ f(x, u+eps*e_j) - f(x, u-eps*e_j) ] / (2*eps)
%    eps = 1e-5  (good tradeoff: avoids cancellation and truncation error)
%
%  Inputs:
%    x_trim [12x1] - trim state (Euler form, from compute_trim)
%    u_trim [4x1]  - trim inputs (from compute_trim)
%    params        - struct from mav_params()
%
%  Outputs:
%    A  [12x12] - state Jacobian
%    B  [12x4]  - input Jacobian
%
%  Usage:
%    [A, B] = linearize(x_trim, u_trim, params);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 5
% =========================================================================

function [A, B] = linearize(x_trim, u_trim, params)

    eps       = 1e-5;
    n_states  = 12;
    n_inputs  = 4;

    % -----------------------------------------------------------------------
    % A MATRIX  (df/dx)
    % -----------------------------------------------------------------------
    A = zeros(n_states, n_states);

    for i = 1:n_states
        x_plus  = x_trim;  x_plus(i)  = x_plus(i)  + eps;
        x_minus = x_trim;  x_minus(i) = x_minus(i) - eps;

        fp = euler_dynamics(x_plus,  u_trim, params);
        fm = euler_dynamics(x_minus, u_trim, params);

        A(:, i) = (fp - fm) / (2 * eps);
    end

    % -----------------------------------------------------------------------
    % B MATRIX  (df/du)
    % -----------------------------------------------------------------------
    B = zeros(n_states, n_inputs);

    for j = 1:n_inputs
        u_plus  = u_trim;  u_plus(j)  = u_plus(j)  + eps;
        u_minus = u_trim;  u_minus(j) = u_minus(j) - eps;

        fp = euler_dynamics(x_trim, u_plus,  params);
        fm = euler_dynamics(x_trim, u_minus, params);

        B(:, j) = (fp - fm) / (2 * eps);
    end

end

% =========================================================================
%  EULER_DYNAMICS  (private helper)
%  Evaluate the 12-state Euler right-hand side f(x12, u4) for use in the
%  numerical Jacobian.  Internally converts Euler angles to quaternion to
%  call the existing forces_moments() function.
% =========================================================================
function xdot = euler_dynamics(x12, u4, params)

    % --- Unpack 12-state vector ---
    pn    = x12(1);
    pe    = x12(2);
    pd    = x12(3);
    u     = x12(4);
    v     = x12(5);
    w     = x12(6);
    phi   = x12(7);
    theta = x12(8);
    psi   = x12(9);
    p     = x12(10);
    q     = x12(11);
    r     = x12(12);

    % --- Convert Euler angles to quaternion ---
    e = euler_to_quaternion(phi, theta, psi);   % [e0; e1; e2; e3]

    % --- Build 13-state vector ---
    x13 = [pn; pe; pd; u; v; w; e(1); e(2); e(3); e(4); p; q; r];

    % --- Forces and moments (no wind during linearization) ---
    [fm, ~, ~, ~] = forces_moments(x13, u4, zeros(3,1), zeros(3,1), params);
    Fx    = fm(1);  Fy = fm(2);  Fz = fm(3);
    l_mom = fm(4);  m_mom = fm(5);  n_mom = fm(6);

    % -----------------------------------------------------------------------
    % 1. POSITION KINEMATICS  (NED)
    %    [pn_dot; pe_dot; pd_dot] = R_b^v * [u; v; w]
    %    R_b^v built from Euler angles (body → vehicle)
    % -----------------------------------------------------------------------
    cp = cos(phi);   sp = sin(phi);
    ct = cos(theta); st = sin(theta);
    cps = cos(psi);  sps = sin(psi);

    R_bv = [ct*cps,   sp*st*cps - cp*sps,   cp*st*cps + sp*sps;
            ct*sps,   sp*st*sps + cp*cps,   cp*st*sps - sp*cps;
            -st,      sp*ct,                cp*ct              ];

    pos_dot = R_bv * [u; v; w];

    % -----------------------------------------------------------------------
    % 2. TRANSLATIONAL DYNAMICS  (Newton in body frame)
    % -----------------------------------------------------------------------
    u_dot = r*v - q*w + Fx / params.mass;
    v_dot = p*w - r*u + Fy / params.mass;
    w_dot = q*u - p*v + Fz / params.mass;

    % -----------------------------------------------------------------------
    % 3. EULER ANGLE KINEMATICS
    %    [phi_dot; theta_dot; psi_dot] = T(phi,theta) * [p; q; r]
    %    Singular at theta = ±90° (gimbal lock) — acceptable near trim
    % -----------------------------------------------------------------------
    tt = tan(theta);
    T_euler = [1,   sp*tt,   cp*tt;
               0,   cp,     -sp;
               0,   sp/ct,   cp/ct];

    angle_dot = T_euler * [p; q; r];

    % -----------------------------------------------------------------------
    % 4. ROTATIONAL DYNAMICS  (Euler equations with Gamma constants)
    % -----------------------------------------------------------------------
    p_dot = params.Gamma1*p*q - params.Gamma2*q*r + params.Gamma3*l_mom + params.Gamma4*n_mom;
    q_dot = params.Gamma5*p*r - params.Gamma6*(p^2 - r^2)               + m_mom / params.Jy;
    r_dot = params.Gamma7*p*q - params.Gamma1*q*r + params.Gamma4*l_mom + params.Gamma8*n_mom;

    % --- Pack derivative vector ---
    xdot = [pos_dot;              % pn_dot, pe_dot, pd_dot
            u_dot; v_dot; w_dot;  % translational dynamics
            angle_dot;            % phi_dot, theta_dot, psi_dot
            p_dot; q_dot; r_dot]; % rotational dynamics

end
