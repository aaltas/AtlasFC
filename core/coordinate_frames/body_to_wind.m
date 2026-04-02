function R = body_to_wind(alpha, beta)
% BODY_TO_WIND  Rotation matrix from body frame to wind frame.
%
%   R = body_to_wind(alpha, beta)
%
%   Returns R_b^w(alpha, beta) = (R_w^b)^T such that:
%       p^w = R * p^b
%
%   From Beard & McLain, Ch.2, Slide 15:
%
%   R_b^w = | cosβcosα    sinβ    cosβsinα |
%            |-sinβcosα    cosβ   -sinβsinα |
%            |-sinα         0      cosα     |
%
%   INPUTS:
%     alpha [rad] - angle of attack
%     beta  [rad] - sideslip angle
%
%   OUTPUT:
%     R     [3x3] - rotation matrix (body to wind)
%
%   See also: wind_to_body, airspeed_alpha_beta

R = wind_to_body(alpha, beta)';

end
