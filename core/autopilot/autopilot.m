% =========================================================================
%  AUTOPILOT - Top-level autopilot combining all control channels
% =========================================================================
%  Implements the complete successive loop closure autopilot from
%  Beard & McLain Ch.6.  Call this once per simulation time step.
%
%  CONTROL HIERARCHY:
%    LATERAL  — Course → Roll → Aileron
%      chi_ref → course_hold → phi_ref → roll_hold → delta_a
%
%    LONGITUDINAL — Altitude → Pitch → Elevator
%                   Airspeed → Throttle
%      h_ref   → altitude_hold → theta_ref → pitch_hold → delta_e
%      Va_ref  → airspeed_hold → delta_t
%
%    RUDDER: δr = δr_trim  (coordinated turn assumed; Dutch roll not
%                           separately controlled in this chapter)
%
%  INTEGRATOR STATE:
%    This function is stateless — it receives the integrator state
%    (ap_state) and returns the updated state.  Caller must preserve
%    ap_state between time steps.
%
%    ap_state fields:
%      .Va_int  - airspeed error integral [m·s]
%
%  Inputs:
%    cmd        - struct with fields:
%                   .Va_c   [m/s]  commanded airspeed
%                   .h_c    [m]    commanded altitude (positive up)
%                   .chi_c  [rad]  commanded course angle
%    x12        - 12-state vector [pn,pe,pd,u,v,w,phi,theta,psi,p,q,r]
%    u_trim     - trim inputs [delta_e, delta_a, delta_r, delta_t]
%    gains      - struct from autopilot_gains()
%    ap_state   - integrator state struct (see above)
%    dt         - simulation time step [s]
%
%  Outputs:
%    u_cmd      - [4x1] control inputs [delta_e, delta_a, delta_r, delta_t]
%    ap_state   - updated integrator state
%
%  Usage:
%    ap_state = struct('Va_int', 0);
%    [u, ap_state] = autopilot(cmd, x12, u_trim, gains, ap_state, dt);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 6
% =========================================================================

function [u_cmd, ap_state] = autopilot(cmd, x12, u_trim, gains, ap_state, dt)

    % -----------------------------------------------------------------------
    % 1. EXTRACT STATE VARIABLES
    % -----------------------------------------------------------------------
    phi   = x12(7);     % roll angle    [rad]
    theta = x12(8);     % pitch angle   [rad]
    psi   = x12(9);     % yaw angle     [rad]
    p     = x12(10);    % roll rate     [rad/s]
    q     = x12(11);    % pitch rate    [rad/s]

    u_b   = x12(4);     % body x velocity [m/s]
    v_b   = x12(5);     % body y velocity [m/s]
    w_b   = x12(6);     % body z velocity [m/s]

    Va    = sqrt(u_b^2 + v_b^2 + w_b^2);   % airspeed [m/s]
    h     = -x12(3);                         % altitude [m]  (h = −pd)

    % Course angle: for zero-wind case, course ≈ heading
    % (for wind, should use ground-speed direction, but simplified here)
    chi   = psi;

    % -----------------------------------------------------------------------
    % 2. LATERAL CHANNELS
    % -----------------------------------------------------------------------
    % Course hold → desired roll angle
    phi_c = course_hold(cmd.chi_c, chi, gains);

    % Roll hold → aileron command
    delta_a = roll_hold(phi_c, phi, p, u_trim(2), gains);

    % -----------------------------------------------------------------------
    % 3. LONGITUDINAL CHANNELS
    % -----------------------------------------------------------------------
    % Altitude hold → desired pitch angle
    theta_c = altitude_hold(cmd.h_c, h, gains.theta_star, gains);

    % Pitch hold → elevator command
    delta_e = pitch_hold(theta_c, theta, q, u_trim(1), gains);

    % Airspeed hold → throttle command
    [delta_t, ap_state.Va_int] = airspeed_hold(cmd.Va_c, Va, u_trim(4), ...
                                                gains, ap_state.Va_int, dt);

    % -----------------------------------------------------------------------
    % 4. RUDDER (trim value — not separately controlled in Ch.6)
    % -----------------------------------------------------------------------
    delta_r = u_trim(3);

    % -----------------------------------------------------------------------
    % 5. PACK OUTPUT
    % -----------------------------------------------------------------------
    u_cmd = [delta_e; delta_a; delta_r; delta_t];

end
