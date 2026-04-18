% =========================================================================
%  GRAVITY_FORCE - Gravity vector in body frame
% =========================================================================
%  Computes the gravity force vector expressed in the body frame.
%
%  In the NED inertial frame, gravity is [0; 0; m*g].
%  We rotate to body frame using R_v^b (inertial → body).
%
%  Using quaternion (Beard & McLain Ch.4, Slide 5):
%
%    f_g^b = m*g * [ 2*(ex*ez - ey*e0)    ]
%                  [ 2*(ey*ez + ex*e0)    ]
%                  [ ez^2 + e0^2 - ex^2 - ey^2 ]
%
%  (This is the third column of R_v^b, scaled by m*g)
%
%  Inputs:
%    e      - quaternion [e0; e1; e2; e3]  (unit norm)
%    params - struct from mav_params()
%
%  Outputs:
%    fg - [3x1] gravity force in body frame [N]
%         fg(1) = fx_grav
%         fg(2) = fy_grav
%         fg(3) = fz_grav
%
%  Usage:
%    fg = gravity_force(e, params)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 4, Slide 5
% =========================================================================

function fg = gravity_force(e, params)

    mg   = params.mass * params.gravity;

    % Rotate gravity from inertial (NED) to body frame
    R_vb = quaternion_to_rotation(e);
    fg   = R_vb * [0; 0; mg];

end
