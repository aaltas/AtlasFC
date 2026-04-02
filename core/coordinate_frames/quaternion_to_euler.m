function [phi, theta, psi] = quaternion_to_euler(q)
% QUATERNION_TO_EULER  Convert unit quaternion to 3-2-1 Euler angles.
%
%   [phi, theta, psi] = quaternion_to_euler(q)
%
%   Recovers roll-pitch-yaw angles from a unit quaternion by first
%   computing the rotation matrix, then extracting Euler angles.
%   This avoids directly inverting quaternion formulas (which are messy)
%   and reuses the tested rotation_to_euler function.
%
%   INPUT:
%     q     [4x1] - unit quaternion [e0; e1; e2; e3]
%
%   OUTPUTS:
%     phi   [rad] - roll
%     theta [rad] - pitch
%     psi   [rad] - yaw
%
%   See also: euler_to_quaternion, quaternion_to_rotation, rotation_to_euler

R = quaternion_to_rotation(q);
[phi, theta, psi] = rotation_to_euler(R);

end
