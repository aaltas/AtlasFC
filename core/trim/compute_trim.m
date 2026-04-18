% =========================================================================
%  COMPUTE_TRIM - Find trim state and inputs for a desired flight condition
% =========================================================================
%  Uses numerical optimization (fminsearch) to find the equilibrium state
%  x* and control inputs u* such that f(x*, u*) ≈ 0 for the requested
%  flight condition (Va*, gamma*, R*).
%
%  Trim optimization variables (straight-level flight):
%    p_opt = [alpha, delta_e, delta_t, delta_a]   (4 unknowns)
%
%  NOTE: delta_a is included to balance the motor reaction torque Q_p,
%  which creates a rolling moment that must be compensated at trim.
%
%  Trim is defined by:
%    Va_star    - desired airspeed [m/s]
%    gamma_star - desired flight-path angle [rad]  (0 = level, + = climb)
%    R_star     - desired turn radius [m]           (inf = straight)
%
%  Outputs:
%    x_trim [12x1] - trim state in 12-state Euler form
%                    [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
%    u_trim [4x1]  - trim control inputs
%                    [delta_e, delta_a, delta_r, delta_t]
%    info          - struct with diagnostics (alpha, theta, Va, cost, converged)
%
%  Usage:
%    params = mav_params();
%    [x_trim, u_trim, info] = compute_trim(25, 0, inf, params);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 5
% =========================================================================

function [x_trim, u_trim, info] = compute_trim(Va_star, gamma_star, R_star, params)

    % -----------------------------------------------------------------------
    % 1. SET UP OPTIMIZER
    % -----------------------------------------------------------------------
    % Physics-based initial guess
    q_bar     = 0.5 * params.rho * Va_star^2;
    CL_needed = (params.mass * params.gravity * cos(gamma_star)) / (q_bar * params.S_wing);
    alpha_g   = max(0.01, (CL_needed - params.C_L_0) / params.C_L_alpha);
    delta_e_g = -(params.C_m_0 + params.C_m_alpha * alpha_g) / params.C_m_delta_e;
    x0 = [alpha_g;    % alpha   [rad]
          delta_e_g;  % delta_e [rad]
          0.75;       % delta_t [-]  (start higher — low throttle gives near-zero thrust at Va=25)
          0.00];      % delta_a [rad]

    % Cost function wrapper
    cost_fn = @(x_opt) trim_residuals(x_opt, Va_star, gamma_star, R_star, params);

    options = optimset('TolX',       1e-10, ...
                       'TolFun',     1e-10, ...
                       'MaxFunEvals', 10000, ...
                       'MaxIter',    10000, ...
                       'Display',    'off');

    [x_opt, cost] = fminsearch(cost_fn, x0, options);

    % -----------------------------------------------------------------------
    % 2. EXTRACT TRIM VARIABLES
    % -----------------------------------------------------------------------
    alpha   = x_opt(1);
    delta_e = x_opt(2);
    delta_t = x_opt(3);
    delta_a = x_opt(4);

    % Clamp to physical limits
    delta_t = max(0, min(1, delta_t));

    % -----------------------------------------------------------------------
    % 3. BUILD TRIM STATE (12-state Euler representation)
    % -----------------------------------------------------------------------
    beta  = 0;          % no sideslip (straight flight)
    phi   = 0;          % wings level
    theta = alpha + gamma_star;   % pitch = AoA + FPA
    psi   = 0;          % reference heading

    % Body velocities
    u_b = Va_star * cos(alpha) * cos(beta);
    v_b = Va_star * sin(beta);
    w_b = Va_star * sin(alpha) * cos(beta);

    % Angular rates
    if isinf(R_star)
        p_trim = 0;  q_trim = 0;  r_trim = 0;
    else
        p_trim = -(Va_star / R_star) * sin(theta);
        q_trim =  (Va_star / R_star) * sin(phi) * cos(theta);
        r_trim =  (Va_star / R_star) * cos(phi) * cos(theta);
    end

    % 12-state Euler trim vector
    x_trim = [0;         % pn  [m]
              0;         % pe  [m]
              params.pd0;% pd  [m]  (initial altitude from params)
              u_b;       % u   [m/s]
              v_b;       % v   [m/s]
              w_b;       % w   [m/s]
              phi;       % phi   [rad]
              theta;     % theta [rad]
              psi;       % psi   [rad]
              p_trim;    % p  [rad/s]
              q_trim;    % q  [rad/s]
              r_trim];   % r  [rad/s]

    % -----------------------------------------------------------------------
    % 4. TRIM CONTROL INPUTS
    % -----------------------------------------------------------------------
    u_trim = [delta_e;   % elevator [rad]
              delta_a;  % aileron  [rad]  (balances motor reaction torque)
              0;        % rudder   [rad]
              delta_t]; % throttle [-]

    % -----------------------------------------------------------------------
    % 5. DIAGNOSTICS
    % -----------------------------------------------------------------------
    % Evaluate individual residuals at solution for debugging
    e_dbg   = euler_to_quaternion(0, theta, 0);
    x13_dbg = [0; 0; params.pd0; u_b; v_b; w_b; e_dbg; 0; 0; 0];
    u_trim_dbg = [delta_e; delta_a; 0; delta_t];
    [fm_dbg, ~, ~, ~] = forces_moments(x13_dbg, u_trim_dbg, zeros(3,1), zeros(3,1), params);
    xd_dbg  = mav_derivatives(x13_dbg, fm_dbg, params);
    info.u_dot = xd_dbg(4);
    info.w_dot = xd_dbg(6);
    info.q_dot = xd_dbg(12);
    info.p_dot = xd_dbg(11);

    info.alpha     = alpha;
    info.theta     = theta;
    info.Va        = Va_star;
    info.gamma     = gamma_star;
    info.R         = R_star;
    info.cost      = cost;
    info.converged = (cost < 1e-6);

end
