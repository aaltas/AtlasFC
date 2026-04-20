% =========================================================================
%  DUBINS_PARAMETERS - Compute shortest Dubins path between two configurations
% =========================================================================
%  Finds the shortest CSC (Curve-Straight-Curve) Dubins path connecting
%  two UAV configurations (position + heading) respecting minimum radius R.
%
%  COORDINATE CONVENTION  (pn = North, pe = East):
%    Heading chi : measured from +pn, clockwise toward +pe
%    Forward     : [cos(chi); sin(chi)]  in (pn, pe)
%    Right perp  : [-sin(chi); cos(chi)] in (pn, pe)   (90° CW from fwd)
%    Left  perp  : [ sin(chi); -cos(chi)] in (pn, pe)  (90° CCW from fwd)
%
%  CIRCLE CENTRES:
%    CW  (right) circle: c_cw = p + R * [-sin(chi);  cos(chi)]
%    CCW (left)  circle: c_ccw= p + R * [ sin(chi); -cos(chi)]
%
%  LAMBDA SIGN CONVENTION  (must match follow_orbit.m):
%    lambda = +1  →  CW  (right turn)   — follow_orbit commands chi = phi_c + pi/2
%    lambda = -1  →  CCW (left  turn)   — follow_orbit commands chi = phi_c - pi/2
%
%    NOTE: follow_orbit's documentation says "+1=CCW", but the actual
%    guidance law (chi_c = phi_c + lambda*pi/2) makes lambda=+1 produce
%    clockwise flight. We follow the CODE, not the comment.
%
%  FOUR CSC PATH TYPES:
%    RSR  — Right (CW)  arc, Straight, Right (CW)  arc   lambda_s=+1, lambda_e=+1
%    LSL  — Left  (CCW) arc, Straight, Left  (CCW) arc   lambda_s=-1, lambda_e=-1
%    RSL  — Right (CW)  arc, Straight, Left  (CCW) arc   lambda_s=+1, lambda_e=-1
%    LSR  — Left  (CCW) arc, Straight, Right (CW)  arc   lambda_s=-1, lambda_e=+1
%
%  INPUTS:
%    ps    [3×1]  Start position  [pn; pe; pd]
%    chi_s [1×1]  Start heading   [rad]
%    pe_in [3×1]  End   position  [pn; pe; pd]
%    chi_e [1×1]  End   heading   [rad]
%    R     [1×1]  Minimum turning radius [m]
%
%  OUTPUTS:
%    dp struct:
%      .type      'RSR' | 'LSL' | 'RSL' | 'LSR'
%      .L         total path length [m]
%      .L1 .L2 .L3  segment lengths [m]
%      .c_s       [3×1] start circle centre
%      .lambda_s  +1 (CW) or -1 (CCW) for start circle
%      .c_e       [3×1] end circle centre
%      .lambda_e  +1 (CW) or -1 (CCW) for end circle
%      .z1        [3×1] arc-1 → straight  transition point
%      .z2        [3×1] straight → arc-2  transition point
%      .z3        [3×1] = pe_in  (endpoint / switching reference)
%      .q         [3×1] unit direction of straight segment (3-D, pd=0)
%      .R         [1×1] radius used
%
%  REFERENCE:
%    Beard & McLain, "Small Unmanned Aircraft", Ch. 10.
%
%  Author : AtlasFC
% =========================================================================

function dp = dubins_parameters(ps, chi_s, pe_in, chi_e, R)

    ps2 = ps(1:2);
    pe2 = pe_in(1:2);
    h_s = ps(3);
    h_e = pe_in(3);

    % -----------------------------------------------------------------------
    %  Circle centres  (see sign convention in header)
    % -----------------------------------------------------------------------
    c_rs = ps2 + R * [-sin(chi_s);  cos(chi_s)];   % right / CW  start
    c_ls = ps2 + R * [ sin(chi_s); -cos(chi_s)];   % left  / CCW start
    c_re = pe2 + R * [-sin(chi_e);  cos(chi_e)];   % right / CW  end
    c_le = pe2 + R * [ sin(chi_e); -cos(chi_e)];   % left  / CCW end

    % -----------------------------------------------------------------------
    %  Evaluate all four candidate paths
    % -----------------------------------------------------------------------
    dp_rsr = csc_rsr(chi_s, chi_e, R, c_rs, c_re, h_s, h_e, pe_in);
    dp_lsl = csc_lsl(chi_s, chi_e, R, c_ls, c_le, h_s, h_e, pe_in);
    dp_rsl = csc_rsl(chi_s, chi_e, R, c_rs, c_le, h_s, h_e, pe_in);
    dp_lsr = csc_lsr(chi_s, chi_e, R, c_ls, c_re, h_s, h_e, pe_in);

    % -----------------------------------------------------------------------
    %  Select shortest valid path
    % -----------------------------------------------------------------------
    candidates = {dp_rsr, dp_lsl, dp_rsl, dp_lsr};
    best_L = inf;
    dp = [];
    for i = 1:4
        if ~isempty(candidates{i}) && candidates{i}.L < best_L
            best_L = candidates{i}.L;
            dp = candidates{i};
        end
    end

    if isempty(dp)
        error('dubins_parameters: no valid path found (R=%.1f m too large?)', R);
    end
    dp.R  = R;
    dp.z3 = pe_in;

end

% =========================================================================
%  Local sub-functions — one per CSC type
% =========================================================================
%
%  KEY FORMULAS (derived from coordinate convention in header):
%
%  Tangent point z on CW  circle c_cw  with heading θ:
%    z = c_cw  + R * [ sin(θ); -cos(θ)]
%    (because c_cw  = z + R*[-sin(θ); cos(θ)] by definition of right-circle centre)
%
%  Tangent point z on CCW circle c_ccw with heading θ:
%    z = c_ccw + R * [-sin(θ);  cos(θ)]
%
%  Arc lengths:
%    CW  arc from chi_a to chi_b : R * mod(chi_b - chi_a, 2π)
%    CCW arc from chi_a to chi_b : R * mod(chi_a - chi_b, 2π)
%
% =========================================================================

% -------------------------------------------------------------------------
function dp = csc_rsr(chi_s, chi_e, R, c_rs, c_re, h_s, h_e, pe_in)
    % Right-Straight-Right  (both CW, lambda = +1)
    d = c_re - c_rs;
    ell = norm(d);
    theta = atan2(d(2), d(1));

    z1_2d = c_rs + R * [ sin(theta); -cos(theta)];  % tangent on CW start circle
    z2_2d = c_re + R * [ sin(theta); -cos(theta)];  % tangent on CW end   circle

    L1 = R * mod(theta  - chi_s, 2*pi);   % CW arc
    L2 = ell;
    L3 = R * mod(chi_e  - theta,  2*pi);  % CW arc

    dp = make_dp('RSR', L1, L2, L3, ...
                 [c_rs; h_s], +1, [c_re; h_e], +1, ...
                 [z1_2d; h_s], [z2_2d; h_e], ...
                 [cos(theta); sin(theta); 0], pe_in);
end

% -------------------------------------------------------------------------
function dp = csc_lsl(chi_s, chi_e, R, c_ls, c_le, h_s, h_e, pe_in)
    % Left-Straight-Left  (both CCW, lambda = -1)
    d = c_le - c_ls;
    ell = norm(d);
    theta = atan2(d(2), d(1));

    z1_2d = c_ls + R * [-sin(theta);  cos(theta)];  % tangent on CCW start circle
    z2_2d = c_le + R * [-sin(theta);  cos(theta)];  % tangent on CCW end   circle

    L1 = R * mod(chi_s - theta,  2*pi);  % CCW arc
    L2 = ell;
    L3 = R * mod(theta  - chi_e, 2*pi); % CCW arc

    dp = make_dp('LSL', L1, L2, L3, ...
                 [c_ls; h_s], -1, [c_le; h_e], -1, ...
                 [z1_2d; h_s], [z2_2d; h_e], ...
                 [cos(theta); sin(theta); 0], pe_in);
end

% -------------------------------------------------------------------------
function dp = csc_rsl(chi_s, chi_e, R, c_rs, c_le, h_s, h_e, pe_in)
    % Right-Straight-Left  (CW start, CCW end — internal tangent)
    d = c_le - c_rs;
    ell = norm(d);
    if ell < 2*R
        dp = [];   % circles too close — no valid RSL path
        return;
    end
    theta = atan2(d(2), d(1));
    beta  = asin(2*R / ell);
    theta_q = theta + beta;              % tangent heading  (corrected sign)

    z1_2d = c_rs + R * [ sin(theta_q); -cos(theta_q)];  % on CW  circle
    z2_2d = c_le + R * [-sin(theta_q);  cos(theta_q)];  % on CCW circle

    L1 = R * mod(theta_q - chi_s, 2*pi);  % CW  arc
    L2 = sqrt(ell^2 - 4*R^2);
    L3 = R * mod(theta_q - chi_e, 2*pi); % CCW arc

    dp = make_dp('RSL', L1, L2, L3, ...
                 [c_rs; h_s], +1, [c_le; h_e], -1, ...
                 [z1_2d; h_s], [z2_2d; h_e], ...
                 [cos(theta_q); sin(theta_q); 0], pe_in);
end

% -------------------------------------------------------------------------
function dp = csc_lsr(chi_s, chi_e, R, c_ls, c_re, h_s, h_e, pe_in)
    % Left-Straight-Right  (CCW start, CW end — internal tangent)
    d = c_re - c_ls;
    ell = norm(d);
    if ell < 2*R
        dp = [];
        return;
    end
    theta = atan2(d(2), d(1));
    beta  = asin(2*R / ell);
    theta_q = theta - beta;              % tangent heading  (corrected sign)

    z1_2d = c_ls + R * [-sin(theta_q);  cos(theta_q)];  % on CCW circle
    z2_2d = c_re + R * [ sin(theta_q); -cos(theta_q)];  % on CW  circle

    L1 = R * mod(chi_s   - theta_q, 2*pi);  % CCW arc
    L2 = sqrt(ell^2 - 4*R^2);
    L3 = R * mod(chi_e   - theta_q, 2*pi); % CW  arc

    dp = make_dp('LSR', L1, L2, L3, ...
                 [c_ls; h_s], -1, [c_re; h_e], +1, ...
                 [z1_2d; h_s], [z2_2d; h_e], ...
                 [cos(theta_q); sin(theta_q); 0], pe_in);
end

% -------------------------------------------------------------------------
function dp = make_dp(type, L1, L2, L3, c_s, lam_s, c_e, lam_e, z1, z2, q, pe_in)
    dp.type     = type;
    dp.L        = L1 + L2 + L3;
    dp.L1       = L1;
    dp.L2       = L2;
    dp.L3       = L3;
    dp.c_s      = c_s;
    dp.lambda_s = lam_s;
    dp.c_e      = c_e;
    dp.lambda_e = lam_e;
    dp.z1       = z1;
    dp.z2       = z2;
    dp.z3       = pe_in;
    dp.q        = q;
end
