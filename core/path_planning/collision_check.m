% =========================================================================
%  COLLISION_CHECK - Check if a path segment collides with map obstacles
% =========================================================================
%  Returns true if the segment from p1 to p2 is COLLISION FREE (clear path).
%  Returns false if any obstacle is hit.
%
%  Obstacle model: vertical cylinders, each defined by:
%    [center_n, center_e, radius, height]
%  A segment violates an obstacle if it passes within `radius` meters of
%  the cylinder axis AND the flight altitude is below `height`.
%
%  ALGORITHM:
%    For each cylinder obstacle:
%      1. Project segment onto horizontal plane
%      2. Compute minimum distance from cylinder center to line segment
%      3. If min_dist < radius AND segment altitude <= height → collision
%
%  INPUTS:
%    p1        [3×1]    Start of segment [pn, pe, pd]
%    p2        [3×1]    End of segment   [pn, pe, pd]
%    obstacles [M×4]    Obstacle matrix: [cn, ce, radius, height] per row
%
%  OUTPUTS:
%    is_clear  [bool]   true = no collision, false = collision detected
%
%  REFERENCE:
%    Beard, R.W. & McLain, T.W. (2012). Small Unmanned Aircraft: Theory
%    and Practice. Princeton University Press. Chapter 11.
%
%  Author : AtlasFC
% =========================================================================

function is_clear = collision_check(p1, p2, obstacles)

    is_clear = true;

    if isempty(obstacles)
        return;
    end

    % Horizontal components of segment
    n1 = p1(1);  e1 = p1(2);  d1 = p1(3);
    n2 = p2(1);  e2 = p2(2);  d2 = p2(3);

    % Segment vector (horizontal)
    dn = n2 - n1;
    de = e2 - e1;
    seg_len_sq = dn^2 + de^2;

    for i = 1:size(obstacles, 1)
        cn     = obstacles(i, 1);
        ce     = obstacles(i, 2);
        radius = obstacles(i, 3);
        height = obstacles(i, 4);   % obstacle top altitude AGL (positive up)

        % --- Altitude check ---
        % pd is negative-up (NED), so altitude h = -pd
        % Flight altitudes along segment range from -d1 to -d2
        h_min = min(-d1, -d2);
        % If UAV always flies above obstacle top: no collision possible
        if h_min > height
            continue;
        end

        % --- Horizontal distance from cylinder axis to segment ---
        % Vector from p1 to cylinder center (horizontal only)
        wn = cn - n1;
        we = ce - e1;

        if seg_len_sq < 1e-10
            % Degenerate segment: check distance from p1 to cylinder
            dist_sq = wn^2 + we^2;
        else
            % Parameter t* of closest point on segment to cylinder center
            t_star = (wn*dn + we*de) / seg_len_sq;
            t_star = max(0.0, min(1.0, t_star));   % clamp to [0,1]

            % Closest point on segment
            closest_n = n1 + t_star * dn;
            closest_e = e1 + t_star * de;

            % Distance from cylinder axis to closest point
            dist_sq = (closest_n - cn)^2 + (closest_e - ce)^2;
        end

        if dist_sq < radius^2
            is_clear = false;
            return;
        end
    end

end
