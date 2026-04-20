% =========================================================================
%  ALTITUDE_HOLD - Altitude hold via pitch angle command
% =========================================================================
%  Outer longitudinal loop: commands pitch angle to achieve desired altitude.
%  The inner pitch_hold loop is assumed to track θ_c perfectly (timescale
%  separation: ωn_h << ωn_θ).
%
%  Simplified plant:  ḣ ≈ Va*(θ − α*)
%  (altitude rate is approximately Va times the flight path angle)
%
%  Control law (P with zone logic + saturation):
%    Zone 1 — CLIMB/DESCEND  (|e_h| > h_zone):
%      θ_c = θ_trim + sign(e_h) * θ_max   [full pitch authority]
%    Zone 2 — HOLD  (|e_h| ≤ h_zone):
%      θ_c = θ_trim + kp_h * (h_c − h)   [proportional]
%
%  θ_c is always saturated to [θ_min, θ_max].
%
%  Inputs:
%    h_c        - commanded altitude [m]  (positive = higher)
%    h          - current altitude [m]    (h = −pd)
%    theta_trim - trim pitch angle [rad]
%    gains      - struct from autopilot_gains()
%
%  Output:
%    theta_c    - commanded pitch angle [rad], saturated
%
%  Usage:
%    theta_c = altitude_hold(h_ref, -x12(3), gains.theta_star, gains);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 6
% =========================================================================

function theta_c = altitude_hold(h_c, h, theta_trim, gains)

    % Altitude error (positive = below commanded altitude)
    e_h = h_c - h;

    if abs(e_h) > gains.h_zone
        % Zone 1: large error — use maximum pitch authority
        theta_c = theta_trim + sign(e_h) * gains.theta_max;
    else
        % Zone 2: proportional hold
        theta_c = theta_trim + gains.kp_h * e_h;
    end

    % Clamp to physical pitch limits
    theta_c = max(gains.theta_min, min(gains.theta_max, theta_c));

end
