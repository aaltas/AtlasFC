% =========================================================================
%  AIRSPEED_HOLD - Airspeed (Va) hold via throttle
% =========================================================================
%  Commands throttle to track a desired airspeed using PI control.
%
%  Transfer function:  Va(s)/δt(s) = a_V2 / (s + a_V1)
%
%  Control law (PI with trim feedforward + anti-windup):
%    δt = δt_trim + kp_V*(Va_c − Va) + ki_V*∫(Va_c − Va)
%
%  Anti-windup: integrator frozen when δt is saturated and error
%  would push further into saturation (back-calculation method).
%
%  Inputs:
%    Va_c           - commanded airspeed [m/s]
%    Va             - current airspeed [m/s]
%    delta_t_trim   - trim throttle [-]
%    gains          - struct from autopilot_gains()
%    Va_int         - current integrator state [m·s] (pass 0 initially)
%    dt             - time step [s]
%
%  Outputs:
%    delta_t        - throttle command [-], saturated to [0, 1]
%    Va_int_new     - updated integrator state
%
%  Usage:
%    [delta_t, Va_int] = airspeed_hold(Va_ref, Va, u_trim(4), gains, Va_int, dt);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 6
% =========================================================================

function [delta_t, Va_int_new] = airspeed_hold(Va_c, Va, delta_t_trim, gains, Va_int, dt)

    % Control limits
    DELTA_T_MIN = 0.0;
    DELTA_T_MAX = 1.0;

    % Airspeed error
    e_Va = Va_c - Va;

    % PI law with trim feedforward
    delta_t_unsat = delta_t_trim           ...
                  + gains.kp_V * e_Va      ...
                  + gains.ki_V * Va_int;

    % Saturation
    delta_t = max(DELTA_T_MIN, min(DELTA_T_MAX, delta_t_unsat));

    % Anti-windup: only integrate when unsaturated, or when error
    % would drive output back into bounds
    saturated = (delta_t ~= delta_t_unsat);
    if saturated && (sign(e_Va) == sign(delta_t_unsat - delta_t))
        % Integrator would make saturation worse — freeze it
        Va_int_new = Va_int;
    else
        Va_int_new = Va_int + e_Va * dt;
    end

end
