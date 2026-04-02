function R = wind_to_body(alpha, beta)
% WIND_TO_BODY  Rotation matrix from wind frame to body frame.
%
%   R = wind_to_body(alpha, beta)
%
%   Returns R_w^b(alpha, beta) such that:
%       p^b = R * p^w
%
%   The wind frame is obtained from the body frame by:
%     1. Rotate by alpha (AoA) about y-axis  → stability frame
%     2. Rotate by beta  (sideslip) about z-axis → wind frame
%   So going from wind to body is the REVERSE of that sequence.
%
%   From Beard & McLain, Ch.2, Slide 16:
%
%   R_w^b = | cosβcosα   -sinβcosα   -sinα |
%            | sinβ        cosβ         0   |
%            | cosβsinα   -sinβsinα    cosα |
%
%   PHYSICAL MEANING:
%     The wind frame's x-axis (i^w) points INTO the airspeed vector V_a.
%     Aerodynamic lift is perpendicular to i^w, drag is along -i^w.
%     Expressing aero forces in wind frame and rotating to body frame is
%     the standard approach for computing aerodynamic forces.
%
%   INPUTS:
%     alpha [rad] - angle of attack
%     beta  [rad] - sideslip angle
%
%   OUTPUT:
%     R     [3x3] - rotation matrix (wind to body)
%
%   See also: body_to_wind, stability_to_body, airspeed_alpha_beta

ca = cos(alpha);  sa = sin(alpha);
cb = cos(beta);   sb = sin(beta);

R = [cb*ca,  -sb*ca,  -sa;
     sb,      cb,      0;
     cb*sa,  -sb*sa,   ca];

end
