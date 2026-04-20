% =========================================================================
%  PITCH_HOLD - Pitch angle (theta) hold autopilot
% =========================================================================
%  Inner longitudinal loop: commands elevator to track a desired pitch angle.
%
%  Transfer function:  θ(s)/δe(s) = a_θ3 / (s² + a_θ1·s + a_θ2)
%
%  Control law (PD with trim feedforward):
%    δe = kp_θ*(θ_c − θ) + kd_θ*q + δe_trim
%
%  where:
%    θ_c       = commanded pitch angle [rad]
%    θ         = current pitch angle   [rad]  (x12(8))
%    q         = pitch rate [rad/s]            (x12(11))
%    δe_trim   = trim elevator [rad]
%
%  SIGN CONVENTION (Beard & McLain):
%    a_θ3 < 0  →  positive δe = trailing edge DOWN = nose DOWN moment
%    Therefore kp_θ < 0 for ωn < √a_θ2 (typical autopilot bandwidth ≈ 5 rad/s)
%
%    When θ < θ_c (need to pitch UP):
%      kp_θ*(positive) = negative δe → trailing edge UP → NOSE UP  ✓
%    When q > 0 (pitching UP too fast):
%      kd_θ*q = positive contribution → trailing edge DOWN → nose DOWN damping ✓
%
%  Inputs:
%    theta_c       - commanded pitch angle [rad]
%    theta         - current pitch angle   [rad]
%    q             - current pitch rate    [rad/s]
%    delta_e_trim  - trim elevator [rad]
%    gains         - struct from autopilot_gains()
%
%  Output:
%    delta_e   - elevator command [rad], saturated to ±30°
%
%  Usage:
%    delta_e = pitch_hold(theta_ref, x12(8), x12(11), u_trim(1), gains);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 6
% =========================================================================

function delta_e = pitch_hold(theta_c, theta, q, delta_e_trim, gains)

    % Control limits
    DELTA_E_MAX = deg2rad(30);   % ±30 degrees

    % Pitch angle error
    e_theta = theta_c - theta;

    % PD law (kp < 0 is correct for a_theta3 < 0, see above)
    delta_e = gains.kp_theta * e_theta  ...
            + gains.kd_theta * q        ...
            + delta_e_trim;

    % Saturation
    delta_e = max(-DELTA_E_MAX, min(DELTA_E_MAX, delta_e));

end
