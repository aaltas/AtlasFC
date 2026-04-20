% =========================================================================
%  WAYPOINT_MANAGER - Waypoint sequencing with half-plane switching
% =========================================================================
%  Manages waypoint sequencing and transition logic using half-plane
%  switching (Beard & McLain, Section 9.3).
%
%  Converts a list of waypoints into active path segments (straight lines
%  or orbits). Automatically switches to the next segment when the vehicle
%  crosses a half-plane defined by the subsequent waypoint.
%
%  ALGORITHM:
%    1. Identify current waypoint leg: from wp_i to wp_{i+1}
%    2. Compute switching half-plane normal:
%       n = normalize(wp_{i+2} - wp_{i+1})  if available
%       else n = normalize(wp_{i+1} - wp_i)
%    3. Check switching condition: (pos - wp_{i+1}) · n ≥ 0
%    4. If true and not at final waypoint: increment wp_idx
%    5. Build path struct with current leg
%
%  INPUTS:
%    waypoints [N×4]    Waypoint matrix [pn, pe, pd, Va_c]
%    pos       [3×1]    Current position [pn, pe, pd]
%    state     [struct] Guidance state with fields:
%        .wp_idx [1×1]  Current waypoint leg index (1-based)
%    cfg       [struct] Configuration with fields:
%        .loop_waypoints [bool] If true, loop back to wp_1 after final
%
%  OUTPUTS:
%    path  [struct]     Current path segment:
%        .type   [str]  'line' (straight-line path following)
%        .r      [3×1]  Reference point on path
%        .q      [3×1]  Unit direction vector
%        .Va     [1×1]  Commanded airspeed
%    state [struct]     Updated guidance state (may have incremented wp_idx)
%
%  NOTES:
%    - Waypoints are 1-indexed in MATLAB
%    - wp_idx refers to the START of the current leg
%    - Altitude is commanded from the path slope (follow_straight_line handles it)
%    - Path type is always 'line' (orbit support can be added later)
%
%  REFERENCE:
%    Beard, R.W. & McLain, T.W. (2012). Small Unmanned Aircraft: Theory
%    and Practice. Princeton University Press. Section 9.3.
%
%  Author : AtlasFC
% =========================================================================

function [path, state] = waypoint_manager(waypoints, pos, state, cfg)

    N_wp = size(waypoints, 1);

    % --- Validate waypoint index ---
    if state.wp_idx < 1 || state.wp_idx > N_wp - 1
        state.wp_idx = 1;
    end

    wp_i = state.wp_idx;      % Current leg start (1-indexed)
    wp_next = wp_i + 1;        % Current leg end

    % --- Waypoint positions ---
    wp_curr = waypoints(wp_i, 1:3)';     % [pn, pe, pd]
    wp_curr_va = waypoints(wp_i, 4);     % Airspeed at this waypoint
    wp_next_pos = waypoints(wp_next, 1:3)';
    wp_next_va = waypoints(wp_next, 4);

    % --- Compute half-plane normal for switching (Beard & McLain Alg. 9.2) ---
    %  Use the bisector of the incoming and outgoing leg unit vectors.
    %  This places the switching half-plane PAST the waypoint for any turn
    %  angle, preventing false triggers when the vehicle is still on the
    %  approach to the waypoint.
    %
    %  q_i     = unit vector along current leg  (wp_curr → wp_next)
    %  q_{i+1} = unit vector along next leg     (wp_next → wp_{i+2})
    %  n       = normalize(q_i + q_{i+1})       bisector
    %
    %  For the final leg (no next-next waypoint) fall back to q_i.

    % Current leg direction
    leg_curr = wp_next_pos - wp_curr;
    leg_curr_mag = norm(leg_curr);
    if leg_curr_mag > 1e-6
        q_i = leg_curr / leg_curr_mag;
    else
        q_i = [1; 0; 0];
    end

    if wp_next < N_wp
        % Outgoing leg direction
        wp_next_next = waypoints(wp_next + 1, 1:3)';
        leg_next = wp_next_next - wp_next_pos;
        leg_next_mag = norm(leg_next);
        if leg_next_mag > 1e-6
            q_ip1 = leg_next / leg_next_mag;
        else
            q_ip1 = q_i;
        end
        n = q_i + q_ip1;
    else
        % Final leg: normal along current leg direction
        n = q_i;
    end

    % Normalize bisector
    n_mag = norm(n);
    if n_mag > 1e-6
        n = n / n_mag;
    else
        n = q_i;
    end

    % --- Check half-plane switching condition ---
    pos_rel = pos - wp_next_pos;
    dot_prod = pos_rel(1)*n(1) + pos_rel(2)*n(2) + pos_rel(3)*n(3);

    if dot_prod >= 0
        % Vehicle has crossed the half-plane
        if wp_next < N_wp
            % Advance to next leg
            state.wp_idx = wp_i + 1;
            wp_i = state.wp_idx;
            wp_next = wp_i + 1;
            wp_curr = waypoints(wp_i, 1:3)';
            wp_curr_va = waypoints(wp_i, 4);
            wp_next_pos = waypoints(wp_next, 1:3)';
            wp_next_va = waypoints(wp_next, 4);
        elseif cfg.loop_waypoints
            % Loop back to start
            state.wp_idx = 1;
            wp_i = 1;
            wp_next = 2;
            wp_curr = waypoints(wp_i, 1:3)';
            wp_curr_va = waypoints(wp_i, 4);
            wp_next_pos = waypoints(wp_next, 1:3)';
            wp_next_va = waypoints(wp_next, 4);
        end
        % If not looping and at end: stay on final leg
    end

    % --- Build path struct for current leg (straight line) ---
    path.type = 'line';
    path.r = wp_curr;                          % Reference point (current waypoint)

    % Direction vector
    leg_vec = wp_next_pos - wp_curr;
    leg_mag = sqrt(leg_vec(1)^2 + leg_vec(2)^2 + leg_vec(3)^2);
    if leg_mag > 1e-6
        path.q = leg_vec / leg_mag;             % Unit direction
    else
        path.q = [1; 0; 0];                    % Fallback
    end

    path.Va = wp_next_va;                      % Use next waypoint's airspeed command

end
