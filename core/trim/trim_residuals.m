% =========================================================================
%  TRIM_RESIDUALS - Cost function for trim optimization
% =========================================================================
%  Evaluates how far the aircraft is from trim for a given set of
%  optimization variables.  Called repeatedly by compute_trim().
%
%  At trim ALL body-frame accelerations must vanish:
%    u_dot = r*v - q*w + Fx/m = 0
%    v_dot = p*w - r*u + Fy/m = 0
%    w_dot = q*u - p*v + Fz/m = 0
%    p_dot = Gamma1*p*q - Gamma2*q*r + Gamma3*l + Gamma4*n = 0
%    q_dot = Gamma5*p*r - Gamma6*(p^2-r^2) + m_mom/Jy      = 0
%    r_dot = Gamma7*p*q - Gamma1*q*r + Gamma4*l + Gamma8*n  = 0
%
%  For straight-level flight the problem decouples neatly:
%    - beta = 0, phi = 0  (lateral symmetry)
%    - p = q = r = 0      (no rotation at trim)
%    - theta = alpha + gamma
%    - delta_a = delta_r = 0
%
%  Optimization variables (for straight-level trim):
%    x_opt(1) = alpha   [rad]  angle of attack
%    x_opt(2) = delta_e [rad]  elevator deflection
%    x_opt(3) = delta_t [-]    throttle  (0-1)
%
%  Inputs:
%    x_opt    - [3x1] optimization variables [alpha; delta_e; delta_t]
%    Va_star  - desired airspeed [m/s]
%    gamma_star - desired flight-path angle [rad] (0 = level)
%    R_star   - desired turn radius [m] (inf = straight)
%    params   - struct from mav_params()
%
%  Output:
%    J        - scalar cost = sum of squared accelerations [SI units]
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 5
% =========================================================================

function J = trim_residuals(x_opt, Va_star, gamma_star, R_star, params)

    % --- Unpack optimization variables ---
    alpha   = x_opt(1);
    delta_e = x_opt(2);
    delta_t = x_opt(3);

    % --- Trim geometry (straight flight, symmetric) ---
    beta  = 0;
    phi   = 0;
    psi   = 0;
    theta = alpha + gamma_star;   % flight-path angle geometry

    % --- Body-frame velocity components ---
    u_b = Va_star * cos(alpha) * cos(beta);
    v_b = Va_star * sin(beta);
    w_b = Va_star * sin(alpha) * cos(beta);

    % --- Angular rates at trim ---
    if isinf(R_star)
        p_r = 0;  q_r = 0;  r_r = 0;
    else
        % Coordinated turn formulas (Beard & McLain Eq. 5.4)
        p_r = -(Va_star / R_star) * sin(theta);
        q_r =  (Va_star / R_star) * sin(phi)  * cos(theta);
        r_r =  (Va_star / R_star) * cos(phi)  * cos(theta);
    end

    % --- Build 13-state vector (quaternion representation) ---
    e = euler_to_quaternion(phi, theta, psi);   % [e0; e1; e2; e3]

    x13 = [0;      % pn
           0;      % pe
           params.pd0;  % pd  (use nominal altitude)
           u_b;    % u
           v_b;    % v
           w_b;    % w
           e(1); e(2); e(3); e(4);   % quaternion
           p_r; q_r; r_r];           % angular rates

    % --- Control inputs ---
    delta = [delta_e; 0; 0; delta_t];   % [delta_e, delta_a, delta_r, delta_t]

    % --- Forces and moments (no wind at trim) ---
    [fm, ~, ~, ~] = forces_moments(x13, delta, zeros(3,1), zeros(3,1), params);

    Fx = fm(1);  Fy = fm(2);  Fz = fm(3);
    l  = fm(4);  m_mom = fm(5);  n  = fm(6);

    % --- Translational acceleration residuals ---
    m_mass = params.mass;
    u_dot  = r_r*v_b  - q_r*w_b + Fx / m_mass;
    v_dot  = p_r*w_b  - r_r*u_b + Fy / m_mass;
    w_dot  = q_r*u_b  - p_r*v_b + Fz / m_mass;

    % --- Rotational acceleration residuals (Euler equations) ---
    p_dot = params.Gamma1*p_r*q_r - params.Gamma2*q_r*r_r + params.Gamma3*l  + params.Gamma4*n;
    q_dot = params.Gamma5*p_r*r_r - params.Gamma6*(p_r^2 - r_r^2)            + m_mom / params.Jy;
    r_dot = params.Gamma7*p_r*q_r - params.Gamma1*q_r*r_r + params.Gamma4*l  + params.Gamma8*n;

    % --- Scalar cost (sum of squared residuals) ---
    J = u_dot^2 + v_dot^2 + w_dot^2 + p_dot^2 + q_dot^2 + r_dot^2;

end
