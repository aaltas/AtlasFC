% =========================================================================
%  LIFT_DRAG_COEFFICIENTS - Nonlinear C_L(alpha) and C_D(alpha)
% =========================================================================
%  Computes the nonlinear lift and drag coefficients using the blending
%  function sigma(alpha) that smoothly transitions between the linear
%  aerodynamic model (small AoA) and the flat-plate model (stalled flow).
%
%  From Beard & McLain, Ch.4, Slides 16-20:
%
%  Blending function:
%    sigma(alpha) = (1 + exp(-M*(alpha-alpha0)) + exp(M*(alpha+alpha0)))
%                / ((1+exp(-M*(alpha-alpha0))) * (1+exp(M*(alpha+alpha0))))
%
%  Nonlinear lift coefficient:
%    C_L(alpha) = (1-sigma)*[C_L0 + C_Lalpha*alpha]
%               + sigma * [2*sign(alpha)*sin^2(alpha)*cos(alpha)]
%
%  Nonlinear drag coefficient (parabolic polar, Slide 20):
%    C_D(alpha) = C_Dp + (C_L0 + C_Lalpha*alpha)^2 / (pi*e*AR)
%
%  The linear model is only valid for small angles where flow is attached.
%  The blending function smoothly replaces it with the flat-plate model
%  near and beyond stall (alpha ~ alpha0).
%
%  Inputs:
%    alpha  - angle of attack [rad]
%    params - struct from mav_params()
%
%  Outputs:
%    CL - nonlinear lift coefficient   [-]
%    CD - nonlinear drag coefficient   [-]
%
%  Usage:
%    [CL, CD] = lift_drag_coefficients(alpha, params)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 4, Slides 16-20
% =========================================================================

function [CL, CD] = lift_drag_coefficients(alpha, params)

    M      = params.M;
    alpha0 = params.alpha0;
    CL0    = params.C_L_0;
    CLa    = params.C_L_alpha;
    e_eff  = params.e;          % Oswald efficiency
    AR     = params.AR;         % aspect ratio = b^2 / S_wing

    % --- Blending function sigma(alpha) ---
    % Approaches 0 for |alpha| << alpha0  (linear regime)
    % Approaches 1 for |alpha| >> alpha0  (stalled / flat-plate regime)
    num   = 1 + exp(-M*(alpha - alpha0)) + exp(M*(alpha + alpha0));
    den   = (1 + exp(-M*(alpha - alpha0))) * (1 + exp(M*(alpha + alpha0)));
    sigma = num / den;

    % --- Nonlinear lift coefficient C_L(alpha) ---
    CL_linear    = CL0 + CLa * alpha;
    CL_flatplate = 2 * sign(alpha) * sin(alpha)^2 * cos(alpha);
    CL = (1 - sigma) * CL_linear + sigma * CL_flatplate;

    % --- Nonlinear drag coefficient C_D(alpha) ---
    % Parabolic polar: C_D = C_Dp + CL_linear^2 / (pi*e*AR)
    % C_Dp is the parasitic (zero-lift) drag; here we use C_D_0 as base
    CD = params.C_D_0 + CL_linear^2 / (pi * e_eff * AR);

end
