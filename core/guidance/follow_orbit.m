% =========================================================================
%  FOLLOW_ORBIT - Circular orbit path following guidance
% =========================================================================
%  Implements path following around a circular orbit (Beard & McLain, Eq. 9.8-9.14).
%
%  Computes course command chi_c and altitude command h_c to guide the
%  aircraft around a defined circular path at constant radius.
%
%  ALGORITHM:
%    1. Compute distance from center: d = sqrt((pn-cn)² + (pe-ce)²)
%    2. Compute angle to vehicle: phi_c = atan2(pe-ce, pn-cn)
%    3. Wrap phi_c to align with current course heading
%    4. Course command: chi_c = phi_c + λ*(π/2 + atan(k_orbit*(d-ρ)/ρ))
%    5. Altitude command: h_c = -c_d (orbit center altitude)
%    6. Radial error: e_r = d - ρ
%
%  INPUTS:
%    orbit [struct] Orbit definition with fields:
%        .c     [3×1]  Center position [cn, ce, cd]
%        .rho   [1×1]  Orbit radius [m]
%        .lambda[1×1]  Orbit direction: +1 (CW) or -1 (CCW)
%        .Va    [1×1]  Desired airspeed [m/s]
%    pos   [3×1]    Current position [pn, pe, pd]
%    chi   [1×1]    Current course heading [rad]
%
%  OUTPUTS:
%    chi_c  [1×1]   Commanded course heading [rad]
%    h_c    [1×1]   Commanded altitude [m]
%    e_r    [1×1]   Radial error [m] (positive = outside orbit)
%
%  PARAMETERS:
%    k_orbit        [1]     Orbit-following gain (default 4.0)
%
%  REFERENCE:
%    Beard, R.W. & McLain, T.W. (2012). Small Unmanned Aircraft: Theory
%    and Practice. Princeton University Press. Chapter 9.
%
%  Author : AtlasFC
% =========================================================================

function [chi_c, h_c, e_r] = follow_orbit(orbit, pos, chi)

    % --- Parameters ---
    k_orbit = 4.0;        % Orbit-following gain

    % --- Extract orbit and position ---
    c = orbit.c;          % Orbit center [3×1]
    rho = orbit.rho;      % Orbit radius [m]
    lambda = orbit.lambda; % Direction: +1 = CW (clockwise), -1 = CCW (counter-clockwise)

    pn = pos(1);
    pe = pos(2);

    % --- Compute distance from center ---
    dpn = pn - c(1);
    dpe = pe - c(2);
    d = sqrt(dpn^2 + dpe^2);

    % --- Compute angle to vehicle ---
    phi_c = atan2(dpe, dpn);

    % --- Wrap phi_c to be close to chi (handle ±π wrap) ---
    % Ensure the angle difference is minimal
    chi_diff = chi - phi_c;
    if chi_diff > pi
        phi_c = phi_c + 2*pi;
    elseif chi_diff < -pi
        phi_c = phi_c - 2*pi;
    end

    % --- Course command ---
    % Guidance law: chi_c = phi_c + λ*(π/2 + atan(k_orbit*(d-ρ)/ρ))
    chi_c = phi_c + lambda * (pi/2 + atan(k_orbit * (d - rho) / rho));

    % --- Altitude command (orbit at constant altitude) ---
    h_c = -c(3);

    % --- Radial error for monitoring ---
    e_r = d - rho;

end
