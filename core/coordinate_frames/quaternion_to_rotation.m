function R = quaternion_to_rotation(q)
% QUATERNION_TO_ROTATION  Convert unit quaternion to rotation matrix.
%
%   R = quaternion_to_rotation(q)
%
%   Converts a unit quaternion q = [e0; e1; e2; e3] to the equivalent
%   3x3 Direction Cosine Matrix (inertial to body convention, same as
%   euler_to_rotation).
%
%   FORMULA (from Beard & McLain, Appendix B):
%
%   R = | e0²+e1²-e2²-e3²    2(e1e2+e0e3)      2(e1e3-e0e2) |
%       | 2(e1e2-e0e3)       e0²-e1²+e2²-e3²   2(e2e3+e0e1) |
%       | 2(e1e3+e0e2)       2(e2e3-e0e1)      e0²-e1²-e2²+e3²|
%
%   NOTE: Since ||q||=1, we have e0²+e1²+e2²+e3²=1, so:
%     e0²+e1²-e2²-e3² = 1 - 2(e2²+e3²)   (equivalent forms)
%
%   INPUT:
%     q  [4x1] - unit quaternion [e0; e1; e2; e3]  (scalar first)
%                Will be normalized internally if ||q|| ≠ 1.
%
%   OUTPUT:
%     R  [3x3] - rotation matrix (inertial to body)
%
%   EXAMPLE:
%     phi=0.2; theta=0.1; psi=1.0;
%     q = euler_to_quaternion(phi, theta, psi);
%     R1 = quaternion_to_rotation(q);
%     R2 = euler_to_rotation(phi, theta, psi);
%     max(abs(R1(:)-R2(:)))  % should be near zero
%
%   See also: euler_to_quaternion, quaternion_to_euler, euler_to_rotation

% Normalize (safety)
q = q / norm(q);

e0 = q(1);  e1 = q(2);  e2 = q(3);  e3 = q(4);

R = [e0^2+e1^2-e2^2-e3^2,   2*(e1*e2+e0*e3),       2*(e1*e3-e0*e2);
     2*(e1*e2-e0*e3),        e0^2-e1^2+e2^2-e3^2,   2*(e2*e3+e0*e1);
     2*(e1*e3+e0*e2),        2*(e2*e3-e0*e1),        e0^2-e1^2-e2^2+e3^2];

end
