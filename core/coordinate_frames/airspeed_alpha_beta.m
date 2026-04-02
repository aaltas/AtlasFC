function [Va, alpha, beta] = airspeed_alpha_beta(u_r, v_r, w_r)
% AIRSPEED_ALPHA_BETA  Compute airspeed, angle of attack, and sideslip angle.
%
%   [Va, alpha, beta] = airspeed_alpha_beta(u_r, v_r, w_r)
%
%   Given the airspeed vector expressed in the body frame (relative to
%   surrounding air mass), computes the scalar airspeed Va and the two
%   aerodynamic angles: angle of attack (alpha) and sideslip (beta).
%
%   DEFINITIONS (Beard & McLain, Ch.2, Slides 17-18):
%     V_a^b = [u_r; v_r; w_r]  - airspeed in body frame
%                               = V_g^b - V_wind^b
%                               = [u-u_w; v-v_w; w-w_w]
%
%     Va    = sqrt(u_r² + v_r² + w_r²)     [m/s]  - scalar airspeed
%     alpha = atan2(w_r, u_r)               [rad]  - angle of attack
%     beta  = atan2(v_r, sqrt(u_r²+w_r²))  [rad]  - sideslip angle
%
%   PHYSICAL INTERPRETATION:
%     alpha (α): angle between body x-axis and projection of airspeed onto
%                the xz-plane of the body frame.
%                Positive α → nose pitched up relative to incoming air.
%                Directly related to lift generation.
%
%     beta  (β): angle between airspeed vector and the body xz-plane.
%                Positive β → air coming from the right (starboard).
%                Nominally zero for well-trimmed tailed aircraft.
%
%   RELATIONSHIP TO FRAMES:
%     The stability frame is obtained by rotating the body frame by α.
%     The wind frame is obtained by further rotating by β.
%     See: stability_to_body.m, wind_to_body.m
%
%   INPUTS:
%     u_r  [m/s] - x-body component of airspeed (forward)
%     v_r  [m/s] - y-body component of airspeed (right)
%     w_r  [m/s] - z-body component of airspeed (down)
%
%   OUTPUTS:
%     Va    [m/s] - total airspeed
%     alpha [rad] - angle of attack  (in (-pi, pi])
%     beta  [rad] - sideslip angle   (in (-pi/2, pi/2))
%
%   EXAMPLE:
%     % Pure forward flight, no wind:
%     [Va, alpha, beta] = airspeed_alpha_beta(25, 0, 0);
%     % Va=25, alpha=0, beta=0
%
%     % 5 m/s climb (w_r negative for climb in NED? No, w_r>0 is nose down)
%     [Va, alpha, beta] = airspeed_alpha_beta(25, 0, 2);
%     % alpha = atan2(2,25) ≈ 0.08 rad (slight positive AoA)
%
%   See also: wind_to_body, body_to_wind, stability_to_body

Va    = sqrt(u_r^2 + v_r^2 + w_r^2);

if Va < 1e-6
    % Avoid division by zero at zero airspeed
    alpha = 0;
    beta  = 0;
else
    alpha = atan2(w_r, u_r);
    beta  = atan2(v_r, sqrt(u_r^2 + w_r^2));
end

end
