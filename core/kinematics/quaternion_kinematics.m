% =========================================================================
%  QUATERNION_KINEMATICS - Quaternion rate from body angular rates
% =========================================================================
%  Computes the time derivative of the unit quaternion e = [e0; e1; e2; e3]
%  given body-frame angular rates omega = [p; q; r].
%
%  From Beard & McLain, Chapter 3, Eq. 3.21:
%
%    e_dot = 0.5 * Xi(e) * omega
%
%  where the 4x3 matrix Xi(e) is:
%
%         [-e1  -e2  -e3]
%    Xi = [ e0  -e3   e2]
%         [ e3   e0  -e1]
%         [-e2   e1   e0]
%
%  This formulation is singularity-free and numerically robust.
%  After integration, renormalize: e = e / norm(e)
%
%  Inputs:
%    e     - quaternion [e0; e1; e2; e3], should be unit norm
%    omega - angular rate vector [p; q; r] [rad/s]
%
%  Outputs:
%    e_dot - quaternion rate [4x1]
%
%  Usage:
%    e_dot = quaternion_kinematics(e, omega)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 3
% =========================================================================

function e_dot = quaternion_kinematics(e, omega)

    e0 = e(1);
    e1 = e(2);
    e2 = e(3);
    e3 = e(4);

    p = omega(1);
    q = omega(2);
    r = omega(3);

    % Xi(e) matrix — maps body rates to quaternion rates (Beard & McLain Eq. 3.21)
    Xi = [-e1, -e2, -e3;
           e0, -e3,  e2;
           e3,  e0, -e1;
          -e2,  e1,  e0];

    e_dot = 0.5 * Xi * [p; q; r];

end
