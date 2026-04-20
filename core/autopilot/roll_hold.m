% =========================================================================
%  ROLL_HOLD - Roll angle (phi) hold autopilot
% =========================================================================
%  Inner lateral loop: commands aileron to track a desired roll angle.
%
%  Transfer function:  φ(s)/δa(s) = a_φ2 / [s(s + a_φ1)]
%
%  Control law (PD with trim feedforward):
%    δa = kp_φ*(φ_c − φ) + kd_φ*p + δa_trim
%
%  where:
%    φ_c     = commanded roll angle [rad]
%    φ       = current roll angle [rad]  (x12(7))
%    p       = roll rate [rad/s]          (x12(10))
%    δa_trim = trim aileron from compute_trim [rad]
%
%  The derivative term uses measured roll rate p directly (not numerical
%  differentiation of φ), which avoids noise amplification.
%
%  Inputs:
%    phi_c      - commanded roll angle [rad]
%    phi        - current roll angle   [rad]
%    p          - current roll rate    [rad/s]
%    delta_a_trim - trim aileron [rad]
%    gains      - struct from autopilot_gains()
%
%  Output:
%    delta_a    - aileron command [rad], saturated to ±20°
%
%  Usage:
%    delta_a = roll_hold(phi_ref, x12(7), x12(10), u_trim(2), gains);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 6
% =========================================================================

function delta_a = roll_hold(phi_c, phi, p, delta_a_trim, gains)

    % Control limits
    DELTA_A_MAX = deg2rad(20);   % ±20 degrees

    % Roll angle error
    e_phi = phi_c - phi;

    % PD law (derivative on plant output, not error — avoids derivative kick)
    delta_a = gains.kp_phi * e_phi  ...
            + gains.kd_phi * p      ...
            + delta_a_trim;

    % Saturation
    delta_a = max(-DELTA_A_MAX, min(DELTA_A_MAX, delta_a));

end
