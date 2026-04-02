function R = rot_x(theta)
% ROT_X  Elementary right-handed rotation matrix about the x-axis (i-axis).
%
%   R = rot_x(theta)
%
%   Returns the 3x3 rotation matrix R such that:
%       p^1 = R * p^0
%   where frame-1 is obtained by rotating frame-0 by angle theta
%   (right-hand rule) about the common x-axis.
%
%   From Beard & McLain, Ch.2, Slide 4:
%
%       R = | 1     0        0    |
%           | 0   cos(θ)   sin(θ) |
%           | 0  -sin(θ)   cos(θ) |
%
%   PHYSICAL MEANING:
%     In UAV context this is the ROLL rotation.
%     phi (φ) is the roll angle → R_roll = rot_x(phi)
%
%   PROPERTIES (orthonormal matrix):
%     R^{-1} = R^T       (inverse = transpose)
%     det(R) = 1         (proper rotation, no reflection)
%
%   INPUT:
%     theta  [rad] - rotation angle (scalar)
%
%   OUTPUT:
%     R      [3x3] - rotation matrix (passive/right-handed)
%
%   EXAMPLE:
%     % Rotate a vector 90 degrees about x-axis
%     v = [0; 1; 0];               % points along y
%     R = rot_x(pi/2);
%     v_rotated = R' * v;          % body-to-inertial
%
%   See also: rot_y, rot_z, euler_to_rotation

c = cos(theta);
s = sin(theta);

R = [1,  0,  0;
     0,  c,  s;
     0, -s,  c];

end
