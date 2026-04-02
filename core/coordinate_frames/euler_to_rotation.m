function R = euler_to_rotation(phi, theta, psi)
% EULER_TO_ROTATION  Rotation matrix from inertial (NED) to body frame.
%
%   R = euler_to_rotation(phi, theta, psi)
%
%   Computes the Direction Cosine Matrix (DCM) R_v^b that transforms a
%   vector expressed in the vehicle/inertial NED frame into the body frame:
%
%       p^b = R * p^v
%
%   The rotation is built as a 3-2-1 Euler sequence (ZYX):
%       1. Yaw   about k^v  by psi   → Vehicle-1 frame  (rot_z)
%       2. Pitch about j^v1 by theta → Vehicle-2 frame  (rot_y)
%       3. Roll  about i^v2 by phi   → Body frame        (rot_x)
%
%   So: R_v^b = R_x(phi) * R_y(theta) * R_z(psi)
%
%   From Beard & McLain, Ch.2, Slide 13 — the full expanded matrix:
%
%   R = | cθcψ               cθsψ              -sθ   |
%       | sφsθcψ - cφsψ    sφsθsψ + cφcψ      sφcθ  |
%       | cφsθcψ + sφsψ    cφsθsψ - sφcψ      cφcθ  |
%
%   where c=cos, s=sin, subscripts denote angles (φ,θ,ψ).
%
%   IMPORTANT CONVENTION NOTE:
%     - R transforms FROM inertial TO body  (p^b = R * p^inertial)
%     - R^T transforms FROM body TO inertial (p^inertial = R^T * p^body)
%     - This is the PASSIVE (right-handed) interpretation
%
%   INPUTS:
%     phi   [rad] - roll angle  (rotation about x/i-axis)
%     theta [rad] - pitch angle (rotation about y/j-axis)
%     psi   [rad] - yaw angle   (rotation about z/k-axis, heading)
%
%   OUTPUT:
%     R     [3x3] - Direction Cosine Matrix (inertial to body)
%
%   EXAMPLE:
%     % Level flight heading North:
%     R = euler_to_rotation(0, 0, 0);  % should be eye(3)
%
%     % Nose pointing East (90 deg yaw):
%     R = euler_to_rotation(0, 0, pi/2);
%     g_inertial = [0; 0; 9.81];      % gravity in NED
%     g_body = R * g_inertial;        % gravity felt in body frame
%
%   See also: rotation_to_euler, euler_to_quaternion, rot_x, rot_y, rot_z

cp = cos(phi);    sp = sin(phi);    % c_phi,   s_phi
ct = cos(theta);  st = sin(theta);  % c_theta, s_theta
cs = cos(psi);    ss = sin(psi);    % c_psi,   s_psi

R = [ct*cs,              ct*ss,             -st;
     sp*st*cs - cp*ss,   sp*st*ss + cp*cs,   sp*ct;
     cp*st*cs + sp*ss,   cp*st*ss - sp*cs,   cp*ct];

end
