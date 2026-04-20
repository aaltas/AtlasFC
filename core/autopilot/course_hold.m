% =========================================================================
%  COURSE_HOLD - Course angle (chi) hold via roll angle command
% =========================================================================
%  Outer lateral loop: commands roll angle to achieve desired course.
%  The inner roll_hold loop is assumed to track φ_c perfectly.
%
%  Simplified plant:  χ̇ ≈ (g/Va) * tan(φ) ≈ (g/Va) * φ  (small φ)
%
%  Control law (P with angle-wrap + saturation):
%    φ_c = kp_χ * (χ_c − χ)
%
%  Angle wrapping: course error is wrapped to (−π, π] to avoid
%  300° turns when 60° in the other direction would suffice.
%
%  Inputs:
%    chi_c  - commanded course angle [rad]
%    chi    - current course angle   [rad]  (≈ x12(9) for zero wind)
%    gains  - struct from autopilot_gains()
%
%  Output:
%    phi_c  - commanded roll angle [rad], saturated to ±phi_max
%
%  Usage:
%    phi_c = course_hold(chi_ref, x12(9), gains);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 6
% =========================================================================

function phi_c = course_hold(chi_c, chi, gains)

    % Course error with angle wrap to (−π, π]
    e_chi = chi_c - chi;
    e_chi = atan2(sin(e_chi), cos(e_chi));   % wraps to (−π, π]

    % Proportional command
    phi_c = gains.kp_chi * e_chi;

    % Bank angle saturation
    phi_c = max(-gains.phi_max, min(gains.phi_max, phi_c));

end
