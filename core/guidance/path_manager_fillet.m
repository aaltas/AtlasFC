% =========================================================================
%  PATH_MANAGER_FILLET - Fillet (rounded-corner) waypoint path manager
% =========================================================================
%  Implements the fillet path manager (Beard & McLain, Algorithm 4, Ch.11).
%  Smooth waypoint transitions using small circular arcs at each corner —
%  simpler than Dubins (no R_min heading constraint, just a fixed fillet
%  radius applied at every waypoint turn).
%
%  STATE MACHINE (per waypoint leg i → i+1):
%    State 1 — Straight-line approach to fillet start z1
%      Path   : line from w_i in direction q_i
%      Switch : (pos − z1) · q_i ≥ 0
%      where  : z1 = w_{i+1} − d·q_i,   d = R / tan(varrho/2)
%               varrho = arccos(−q_i · q_{i+1})  (turn half-angle)
%
%    State 2 — Circular fillet arc at w_{i+1}
%      Path   : orbit around c with radius R, direction λ
%      Switch : (pos − z2) · q_{i+1} ≥ 0
%      where  : z2 = w_{i+1} + d·q_{i+1}
%               c  = z1 + R·λ·[−q_i(e); q_i(n)]  (perpendicular to q_i)
%               λ  = sign(q_i × q_{i+1})  (+1 CW, −1 CCW)
%      Action : advance to leg i+1
%
%  FINAL LEG (no next-next waypoint):
%    No fillet — use bisector half-plane switching (same as waypoint_manager).
%
%  INPUTS:
%    waypoints [N×4]    [pn, pe, pd, Va_c]
%    pos       [3×1]    current position [pn; pe; pd]
%    state     struct   guidance state with:
%                .wp_idx     current leg start index
%                .fillet_seg 1 (line) or 2 (orbit)
%    cfg       struct   must contain:
%                .R_fillet   fillet radius [m]
%                .loop_waypoints [bool]
%
%  OUTPUTS:
%    path  struct  'line' or 'orbit' path descriptor
%    state struct  updated guidance state
%
%  REFERENCE:
%    Beard, R.W. & McLain, T.W. (2012). Small Unmanned Aircraft: Theory
%    and Practice. Princeton University Press. Ch. 11, Algorithm 4.
%
%  Author : AtlasFC
% =========================================================================

function [path, state] = path_manager_fillet(waypoints, pos, state, cfg)

    N_wp = size(waypoints, 1);
    R    = cfg.R_fillet;
    pos2 = pos(1:2);

    % --- Clamp index ---
    if state.wp_idx < 1 || state.wp_idx > N_wp - 1
        state.wp_idx = 1;
    end

    wp_i    = state.wp_idx;
    wp_next = wp_i + 1;

    % Waypoint positions
    w_i    = waypoints(wp_i,   1:3)';
    w_next = waypoints(wp_next, 1:3)';

    % Current leg direction q_i
    leg_i  = w_next - w_i;
    lm_i   = norm(leg_i);
    q_i    = ternary(lm_i > 1e-6, leg_i / lm_i, [1;0;0]);

    % -----------------------------------------------------------------------
    %  Fillet geometry — only valid when there IS a next-next waypoint
    % -----------------------------------------------------------------------
    has_nn = (wp_next < N_wp);   % has a waypoint after the next

    if has_nn
        w_nn  = waypoints(wp_next + 1, 1:3)';
        leg_n = w_nn - w_next;
        lm_n  = norm(leg_n);
        q_n   = ternary(lm_n > 1e-6, leg_n / lm_n, q_i);

        % Turn half-angle
        cos_varRho = max(-1, min(1, -dot(q_i(1:2), q_n(1:2))));
        varRho     = acos(cos_varRho);

        % Fillet distance along leg before/after corner
        if abs(tan(varRho / 2)) > 1e-4
            d_f = R / tan(varRho / 2);
        else
            d_f = inf;   % Nearly straight — skip fillet
        end

        % Turning direction: cross product z-component in (pn,pe)
        cross_z = q_i(1)*q_n(2) - q_i(2)*q_n(1);
        lambda  = sign(cross_z);
        if lambda == 0; lambda = 1; end

        % Fillet key points
        z1 = w_next - d_f * q_i;           % Start of arc on incoming leg
        z2 = w_next + d_f * q_n;           % End   of arc on outgoing leg

        % Fillet orbit center: R to the left/right of q_i at z1
        %   right_perp(q_i) in (pn,pe) = [-q_i(e); q_i(n)]
        rp2    = [-q_i(2); q_i(1)];        % right-perpendicular (2D)
        c_2d   = z1(1:2) + R * lambda * rp2;
        c_flt  = [c_2d; w_next(3)];        % orbit center (NED)

        do_fillet = (d_f < inf) && (d_f < norm(w_next(1:2) - w_i(1:2)));
    else
        % Unused when has_nn=false; set defaults to avoid lint warnings
        q_n = q_i;  z1 = w_next;  z2 = w_next;
        c_flt = w_next;  lambda = 1;  do_fillet = false;
    end

    % -----------------------------------------------------------------------
    %  State machine
    % -----------------------------------------------------------------------
    switch state.fillet_seg

        case 1  % ---- Straight-line phase ----
            if has_nn && do_fillet
                % Switch to orbit when UAV crosses half-plane at z1
                hp = dot(pos2 - z1(1:2), q_i(1:2));
                if hp >= 0
                    state.fillet_seg = 2;
                end
            else
                % Final leg (or no fillet): bisector half-plane at w_next
                if wp_next < N_wp
                    q_out = q_n;
                    n     = q_i(1:2) + q_out(1:2);
                else
                    n     = q_i(1:2);
                end
                nm = norm(n);
                if nm > 1e-6; n = n / nm; end

                hp = dot(pos2 - w_next(1:2), n);
                if hp >= 0
                    if wp_next < N_wp
                        state.wp_idx    = wp_next;
                        state.fillet_seg = 1;
                    elseif isfield(cfg,'loop_waypoints') && cfg.loop_waypoints
                        state.wp_idx    = 1;
                        state.fillet_seg = 1;
                    end
                    % else: on final segment, stay put
                end
            end

        case 2  % ---- Fillet orbit phase ----
            hp = dot(pos2 - z2(1:2), q_n(1:2));
            if hp >= 0
                if wp_next < N_wp
                    state.wp_idx    = wp_next;
                    state.fillet_seg = 1;
                elseif isfield(cfg,'loop_waypoints') && cfg.loop_waypoints
                    state.wp_idx    = 1;
                    state.fillet_seg = 1;
                else
                    % Final transition: stay on straight line to last WP
                    state.fillet_seg = 1;
                end
            end

        otherwise
            state.fillet_seg = 1;
    end

    % -----------------------------------------------------------------------
    %  Build path struct for the UPDATED state
    % -----------------------------------------------------------------------
    wp_i    = state.wp_idx;
    wp_next = min(wp_i + 1, N_wp);
    Va_c    = waypoints(wp_next, 4);

    if state.fillet_seg == 2 && has_nn && do_fillet
        % Fillet orbit
        path.type   = 'orbit';
        path.c      = c_flt;
        path.rho    = R;
        path.lambda = lambda;
        path.Va     = Va_c;
    else
        % Straight line on current leg
        path.type = 'line';
        path.r    = waypoints(wp_i, 1:3)';
        leg       = waypoints(wp_next, 1:3)' - path.r;
        lm        = norm(leg);
        path.q    = ternary(lm > 1e-6, leg / lm, [1;0;0]);
        path.Va   = Va_c;
    end

end

% -------------------------------------------------------------------------
function v = ternary(cond, a, b)
    if cond; v = a; else; v = b; end
end
