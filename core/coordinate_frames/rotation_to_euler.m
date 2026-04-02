function [phi, theta, psi] = rotation_to_euler(R)
% ROTATION_TO_EULER  Extract Euler angles from a rotation matrix.
%
%   [phi, theta, psi] = rotation_to_euler(R)
%
%   Recovers the 3-2-1 Euler angles (roll, pitch, yaw) from the
%   Direction Cosine Matrix R (inertial-to-body convention).
%
%   Given the structure of R from euler_to_rotation:
%     R(1,3) = -sin(theta)        → theta = asin(-R(1,3))
%     R(2,3) = sin(phi)*cos(theta)
%     R(3,3) = cos(phi)*cos(theta) → phi  = atan2(R(2,3), R(3,3))
%     R(1,2) = cos(theta)*sin(psi)
%     R(1,1) = cos(theta)*cos(psi) → psi  = atan2(R(1,2), R(1,1))
%
%   SINGULARITY WARNING:
%     When theta = ±90° (gimbal lock), the atan2 formulas for phi and psi
%     become degenerate. In this case only (phi - psi) or (phi + psi) can
%     be determined, not each angle independently. Use quaternions to avoid
%     this singularity in real flight controllers.
%
%   INPUT:
%     R     [3x3] - rotation matrix (inertial to body, from euler_to_rotation)
%
%   OUTPUTS:
%     phi   [rad] - roll angle
%     theta [rad] - pitch angle  (in [-pi/2, pi/2])
%     psi   [rad] - yaw angle    (in (-pi, pi])
%
%   EXAMPLE:
%     phi0 = 0.3; theta0 = 0.1; psi0 = 1.2;
%     R = euler_to_rotation(phi0, theta0, psi0);
%     [phi, theta, psi] = rotation_to_euler(R);
%     % phi, theta, psi should match phi0, theta0, psi0
%
%   See also: euler_to_rotation, quaternion_to_euler

phi   = atan2(R(2,3), R(3,3));
theta = asin(-R(1,3));
psi   = atan2(R(1,2), R(1,1));

end
