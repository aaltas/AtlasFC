function q = euler_to_quaternion(phi, theta, psi)
% EULER_TO_QUATERNION  Convert 3-2-1 Euler angles to unit quaternion.
%
%   q = euler_to_quaternion(phi, theta, psi)
%
%   Converts roll-pitch-yaw Euler angles to a unit quaternion
%   q = [e0; e1; e2; e3] representing the same rotation.
%
%   WHY QUATERNIONS?
%     Euler angles suffer from gimbal lock (singularity at theta = ±90°).
%     Quaternions represent ANY orientation without singularity and are
%     computationally efficient for propagation (used in real flight
%     controllers including Pixhawk/ArduPilot).
%
%   QUATERNION DEFINITION:
%     A rotation of angle Φ about unit axis n̂ = [nx; ny; nz]:
%       e0 = cos(Φ/2)
%       e1 = nx * sin(Φ/2)
%       e2 = ny * sin(Φ/2)
%       e3 = nz * sin(Φ/2)
%
%   For the 3-2-1 (ZYX) Euler sequence, the equivalent quaternion is:
%       e0 = c(ψ/2)c(θ/2)c(φ/2) + s(ψ/2)s(θ/2)s(φ/2)
%       e1 = c(ψ/2)c(θ/2)s(φ/2) - s(ψ/2)s(θ/2)c(φ/2)
%       e2 = c(ψ/2)s(θ/2)c(φ/2) + s(ψ/2)c(θ/2)s(φ/2)
%       e3 = s(ψ/2)c(θ/2)c(φ/2) - c(ψ/2)s(θ/2)s(φ/2)
%
%   PROPERTIES:
%     ||q|| = 1  (unit quaternion, must renormalize periodically)
%     q and -q represent the SAME rotation (double cover of SO(3))
%
%   INPUTS:
%     phi   [rad] - roll
%     theta [rad] - pitch
%     psi   [rad] - yaw
%
%   OUTPUT:
%     q     [4x1] - unit quaternion [e0; e1; e2; e3]  (scalar first)
%
%   EXAMPLE:
%     q = euler_to_quaternion(0, 0, pi/4);  % 45 deg yaw
%     norm(q)  % should be 1.0
%
%   See also: quaternion_to_euler, quaternion_to_rotation, euler_to_rotation

cp = cos(phi/2);    sp = sin(phi/2);
ct = cos(theta/2);  st = sin(theta/2);
cs = cos(psi/2);    ss = sin(psi/2);

e0 = cs*ct*cp + ss*st*sp;
e1 = cs*ct*sp - ss*st*cp;
e2 = cs*st*cp + ss*ct*sp;
e3 = ss*ct*cp - cs*st*sp;

q = [e0; e1; e2; e3];

end
