function R = stability_to_body(alpha)
% STABILITY_TO_BODY  Rotation matrix from stability frame to body frame.
%
%   R = stability_to_body(alpha)
%
%   Returns R_s^b(alpha) such that:
%       p^b = R * p^s
%
%   The stability frame is obtained from the body frame by a rotation of
%   angle alpha (angle of attack) about the common y-axis (j^b = j^s).
%
%   From Beard & McLain, Ch.2, Slide 14:
%
%       R_s^b(α) = | cos(α)   0  -sin(α) |
%                  |   0      1     0    |
%                  | sin(α)   0   cos(α) |
%
%   PHYSICAL MEANING:
%     The stability frame has its x-axis (i^s) pointing into the oncoming
%     airstream, projected onto the body xz-plane. The angle between i^b
%     (body x-axis = nose) and i^s is the angle of attack α.
%     Positive α: air coming from below the nose.
%
%   INPUT:
%     alpha [rad] - angle of attack
%
%   OUTPUT:
%     R     [3x3] - rotation matrix (stability to body)
%
%   See also: body_to_stability, wind_to_body, airspeed_alpha_beta

R = [cos(alpha),  0, -sin(alpha);
     0,            1,  0;
     sin(alpha),   0,  cos(alpha)];

end
