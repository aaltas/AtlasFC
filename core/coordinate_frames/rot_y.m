function R = rot_y(theta)
% ROT_Y  Elementary right-handed rotation matrix about the y-axis (j-axis).
%
%   R = rot_y(theta)
%
%   Returns the 3x3 rotation matrix R such that:
%       p^1 = R * p^0
%   where frame-1 is obtained by rotating frame-0 by angle theta
%   (right-hand rule) about the common y-axis.
%
%   From Beard & McLain, Ch.2, Slide 4:
%
%       R = | cos(θ)   0  -sin(θ) |
%           |   0      1     0    |
%           | sin(θ)   0   cos(θ) |
%
%   PHYSICAL MEANING:
%     In UAV context this is the PITCH rotation.
%     theta (θ) is the pitch angle → R_pitch = rot_y(theta)
%
%   INPUT:
%     theta  [rad] - rotation angle (scalar)
%
%   OUTPUT:
%     R      [3x3] - rotation matrix (passive/right-handed)
%
%   See also: rot_x, rot_z, euler_to_rotation

c = cos(theta);
s = sin(theta);

R = [ c,  0, -s;
      0,  1,  0;
      s,  0,  c];

end
