% =========================================================================
%  SMOOTH_PATH - Post-process RRT path by removing redundant waypoints
% =========================================================================
%  Applies a greedy shortcutting algorithm to the raw RRT waypoint list.
%  Iterates over the path and skips intermediate waypoints whenever the
%  direct segment from the current node to a later node is collision-free.
%
%  This significantly reduces the number of waypoints and straightens the
%  path produced by RRT's random exploration.
%
%  ALGORITHM (greedy shortcut):
%    i = 1
%    while i < N:
%      j = N
%      while j > i+1:
%        if collision_free(path[i], path[j], obstacles):
%          remove path[i+1 .. j-1]
%          break
%        j = j - 1
%      i = i + 1
%
%  INPUTS:
%    path      [N×3]    Waypoint matrix [pn, pe, pd] (one per row)
%    obstacles [M×4]    Obstacle matrix: [cn, ce, radius, height]
%
%  OUTPUTS:
%    smooth    [K×3]    Smoothed waypoint matrix (K <= N)
%
%  REFERENCE:
%    Beard, R.W. & McLain, T.W. (2012). Small Unmanned Aircraft: Theory
%    and Practice. Princeton University Press. Chapter 11.
%
%  Author : AtlasFC
% =========================================================================

function smooth = smooth_path(path, obstacles)

    % Greedy shortcutting
    i = 1;
    while i < size(path, 1)
        j = size(path, 1);
        while j > i + 1
            p1 = path(i, :)';
            p2 = path(j, :)';
            if collision_check(p1, p2, obstacles)
                % Direct segment is clear — remove intermediate points
                path = [path(1:i, :); path(j:end, :)];
                break;
            end
            j = j - 1;
        end
        i = i + 1;
    end

    smooth = path;

end
