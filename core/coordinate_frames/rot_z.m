function R = rot_z(theta)
% ROT_Z  Elementary right-handed rotation matrix about the z-axis (k-axis).
%
%   R = rot_z(theta)
%
%   Returns the 3x3 rotation matrix R such that:
%       p^1 = R * p^0
%   where frame-1 is obtained by rotating frame-0 by angle theta
%   (right-hand rule) about the common z-axis.
%
%   From Beard & McLain, Ch.2, Slide 3:
%
%       R = | cos(θ)   sin(θ)  0 |
%           |-sin(θ)   cos(θ)  0 |
%           |   0        0     1 |
%
%   PHYSICAL MEANING:
%     In UAV context this is the YAW rotation.
%     psi (ψ) is the yaw/heading angle → R_yaw = rot_z(psi)
%     Vehicle frame → Vehicle-1 frame:  p^v1 = rot_z(psi) * p^v
%
%   INPUT:
%     theta  [rad] - rotation angle (scalar)
%
%   OUTPUT:
%     R      [3x3] - rotation matrix (passive/right-handed)
%
%   See also: rot_x, rot_y, euler_to_rotation

c = cos(theta);
s = sin(theta);

R = [ c,  s,  0;
     -s,  c,  0;
      0,  0,  1];

end
