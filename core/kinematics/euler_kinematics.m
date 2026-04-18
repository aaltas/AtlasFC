% =========================================================================
%  EULER_KINEMATICS - Euler angle rates from body angular rates
% =========================================================================
%  Computes the time derivatives of Euler angles [phi, theta, psi] given
%  the body-frame angular rates [p, q, r] and current Euler angles.
%
%  From Beard & McLain, Chapter 3, Eq. 3.14:
%
%   [phi_dot  ]   [1  sin(phi)*tan(theta)  cos(phi)*tan(theta)] [p]
%   [theta_dot] = [0  cos(phi)             -sin(phi)          ] [q]
%   [psi_dot  ]   [0  sin(phi)/cos(theta)  cos(phi)/cos(theta)] [r]
%
%  WARNING: Singular when theta = +/- 90 deg (gimbal lock).
%           Use quaternion_kinematics for singularity-free propagation.
%
%  Inputs:
%    phi   - roll  angle [rad]
%    theta - pitch angle [rad]   (must not be +/- pi/2)
%    psi   - yaw   angle [rad]
%    p     - roll  rate  [rad/s]
%    q     - pitch rate  [rad/s]
%    r     - yaw   rate  [rad/s]
%
%  Outputs:
%    phi_dot   - roll  angle rate  [rad/s]
%    theta_dot - pitch angle rate  [rad/s]
%    psi_dot   - yaw   angle rate  [rad/s]
%
%  Usage:
%    [phi_dot, theta_dot, psi_dot] = euler_kinematics(phi, theta, psi, p, q, r)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 3
% =========================================================================

function [phi_dot, theta_dot, psi_dot] = euler_kinematics(phi, theta, psi, p, q, r)

    cp = cos(phi);
    sp = sin(phi);
    ct = cos(theta);
    tt = tan(theta);

    % Kinematic transformation matrix (Beard & McLain Eq. 3.14)
    T = [1,   sp*tt,   cp*tt;
         0,   cp,     -sp;
         0,   sp/ct,   cp/ct];

    rates = T * [p; q; r];

    phi_dot   = rates(1);
    theta_dot = rates(2);
    psi_dot   = rates(3);

end
