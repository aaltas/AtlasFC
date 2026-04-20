% =========================================================================
%  FOLLOW_STRAIGHT_LINE - Straight-line path following guidance
% =========================================================================
%  Implements path following along a straight line (Beard & McLain, Eq. 9.1-9.7).
%
%  Computes course command chi_c and altitude command h_c to guide the
%  aircraft along a defined straight-line path.
%
%  ALGORITHM:
%    1. Compute path heading: chi_q = atan2(q_e, q_n)
%    2. Compute cross-track error: e_py = -sin(chi_q)*Δpn + cos(chi_q)*Δpe
%    3. Course command: chi_c = chi_q - chi_inf*(2/π)*atan(k_path*e_py)
%    4. Altitude command: h_c = -r_d - s*q_d  (interpolated from path slope)
%
%  INPUTS:
%    path [struct]  Path definition with fields:
%        .r    [3×1]  Reference point on path [pn, pe, pd]
%        .q    [3×1]  Unit direction vector [qn, qe, qd] (normalized)
%        .Va   [1×1]  Desired airspeed [m/s]
%    pos  [3×1]     Current position [pn, pe, pd]
%    chi  [1×1]     Current course heading [rad]
%
%  OUTPUTS:
%    chi_c  [1×1]   Commanded course heading [rad]
%    h_c    [1×1]   Commanded altitude [m]
%    e_py   [1×1]   Cross-track error [m] (positive = left of path)
%
%  PARAMETERS:
%    chi_inf        [rad]   Maximum approach angle (default π/4 = 45°)
%    k_path         [1]     Path-following gain (default 0.025)
%
%  REFERENCE:
%    Beard, R.W. & McLain, T.W. (2012). Small Unmanned Aircraft: Theory
%    and Practice. Princeton University Press. Chapter 9.
%
%  Author : AtlasFC
% =========================================================================

function [chi_c, h_c, e_py] = follow_straight_line(path, pos, chi)

    % --- Parameters ---
    chi_inf = pi/4;       % Maximum approach angle [rad]
    k_path  = 0.025;      % Path-following gain

    % --- Extract path and position ---
    r = path.r;           % Reference point on path [3×1]
    q = path.q;           % Unit direction vector [3×1]

    pn = pos(1);
    pe = pos(2);
    pd = pos(3);

    % --- Compute path heading ---
    chi_q = atan2(q(2), q(1));

    % --- Wrap chi_q to be within ±π of current heading (Algorithm 3, Beard §10.1.2) ---
    while chi_q - chi < -pi;  chi_q = chi_q + 2*pi;  end
    while chi_q - chi >  pi;  chi_q = chi_q - 2*pi;  end

    % --- Compute cross-track error ---
    % e_py = -sin(chi_q)*(pn - r(1)) + cos(chi_q)*(pe - r(2))
    dpn = pn - r(1);
    dpe = pe - r(2);
    e_py = -sin(chi_q)*dpn + cos(chi_q)*dpe;

    % --- Course command with lateral guidance law ---
    chi_c = chi_q - chi_inf * (2/pi) * atan(k_path * e_py);

    % --- Altitude command from path slope ---
    % Distance along path
    s = sqrt(dpn^2 + dpe^2);

    % Altitude interpolation: h_c = -r_d - s*q_d
    % where q_d is vertical component (positive down)
    h_c = -r(3) - s * q(3) / max(sqrt(q(1)^2 + q(2)^2), 0.01);

end
