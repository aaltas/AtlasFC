function R = body_to_stability(alpha)
% BODY_TO_STABILITY  Rotation matrix from body frame to stability frame.
%
%   R = body_to_stability(alpha)
%
%   Returns R_b^s(alpha) = (R_s^b)^T such that:
%       p^s = R * p^b
%
%   INPUT:
%     alpha [rad] - angle of attack
%
%   OUTPUT:
%     R     [3x3] - rotation matrix (body to stability)
%
%   See also: stability_to_body

R = stability_to_body(alpha)';

end
