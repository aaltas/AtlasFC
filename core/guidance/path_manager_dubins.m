% =========================================================================
%  PATH_MANAGER_DUBINS - Dubins-path state machine for waypoint following
% =========================================================================
%  Replaces the straight-line waypoint_manager (Ch9) with Dubins paths.
%  Each waypoint-to-waypoint leg is decomposed into three segments:
%
%    Seg 1 — Initial orbit arc  (follow_orbit around c_s)
%    Seg 2 — Straight segment   (follow_straight_line along q)
%    Seg 3 — Final orbit arc    (follow_orbit around c_e)
%
%  The Dubins path respects minimum turning radius R_min, producing
%  smooth, curved entries and exits at every waypoint.
%
%  SEGMENT TRANSITIONS (Book Algorithm 5, Beard & McLain Ch.11):
%    Seg 1 → 2 : half-plane at z1 (normal q), AFTER UAV has been behind z1
%                (dubins_arc1_neg flag prevents false trigger at new leg start)
%    Seg 2 → 3 : half-plane at z2 (normal q)
%    Seg 3 → next waypoint : half-plane at z3 with OUTGOING leg direction q_out
%                (Book Algorithm 5 — NOT bisector; bisector caused stuck loops)
%
%  INPUTS:
%    waypoints [N×4]    [pn, pe, pd, Va_c]
%    pos       [3×1]    current position [pn; pe; pd]
%    chi       [1×1]    current course heading [rad]
%    state     struct   guidance state (see guidance_init)
%    cfg       struct   must contain cfg.R_min
%
%  OUTPUTS:
%    path  struct  current path segment (type='line' or 'orbit')
%    state struct  updated guidance state
%
%  REFERENCE:
%    Beard & McLain, "Small Unmanned Aircraft", Ch. 10-11.
%    Algorithm 5 (p. 189): Dubins airplane manager.
%
%  Author : AtlasFC
% =========================================================================

function [path, state] = path_manager_dubins(waypoints, pos, chi, state, cfg)

    N_wp  = size(waypoints, 1);
    R     = cfg.R_min;
    pos2d = pos(1:2);

    % --- Clamp waypoint index ---
    if state.wp_idx < 1 || state.wp_idx > N_wp - 1
        state.wp_idx = 1;
    end

    wp_i    = state.wp_idx;
    wp_next = wp_i + 1;

    % --- Initialize arc1_neg flag if missing (backward compat) ---
    if ~isfield(state, 'dubins_arc1_neg')
        state.dubins_arc1_neg = false;
    end

    % --- Compute Dubins path for current leg if not yet done ---
    if isempty(state.dp)
        state.dp = compute_dubins_for_leg(waypoints, wp_i, chi, R);
    end
    dp = state.dp;

    % -----------------------------------------------------------------------
    %  State machine — advance through arc1 → line → arc2 → next leg
    % -----------------------------------------------------------------------
    switch state.dubins_seg

        case 1  % --- Following start arc ---
            % Book Algorithm 5, line 9:
            %   Transition to seg 2 when: (pos - z1) · q >= 0
            %
            % Bug fix: at the START of a new leg the UAV may already be
            % ahead of z1 (hp >= 0) before completing any arc — causing an
            % immediate false skip to seg 2.  We guard against this by
            % requiring that the UAV has at some point been BEHIND z1
            % (hp < 0, i.e. arc1_neg == true) before the transition fires.
            % Exception: if arc1 is negligibly small, skip it outright.

            hp_z1    = dot(pos2d - dp.z1(1:2), dp.q(1:2));
            arc_tiny = dp.L1 < R * 0.05;   % negligible arc → skip immediately

            % Latch: mark that UAV has been behind z1 half-plane
            if hp_z1 < 0
                state.dubins_arc1_neg = true;
            end

            if arc_tiny || (state.dubins_arc1_neg && hp_z1 >= 0)
                state.dubins_seg      = 2;
                state.dubins_arc1_neg = false;   % reset for next leg
            end

        case 2  % --- Following straight segment ---
            hp_z2 = dot(pos2d - dp.z2(1:2), dp.q(1:2));
            if hp_z2 >= 0
                state.dubins_seg = 3;
            end

        case 3  % --- Following end arc ---
            % Book Algorithm 5, line 19:
            %   Transition using half-plane at z3 with normal = q_out
            %   (direction of the NEXT leg, NOT the bisector).
            %   Using the bisector caused the UAV to advance too early and
            %   then orbit forever on the wrong circle.
            q_out = outgoing_dir(waypoints, wp_next, N_wp);
            hp_end = dot(pos2d - dp.z3(1:2), q_out);

            if hp_end >= 0
                if wp_next < N_wp
                    % Advance to next leg
                    old_chi_e = leg_chi_e(waypoints, wp_i);
                    state.wp_idx          = wp_i + 1;
                    state.dubins_seg      = 1;
                    state.dubins_arc1_neg = false;
                    state.dp = compute_dubins_for_leg(waypoints, state.wp_idx, old_chi_e, R);
                elseif isfield(cfg,'loop_waypoints') && cfg.loop_waypoints
                    state.wp_idx          = 1;
                    state.dubins_seg      = 1;
                    state.dubins_arc1_neg = false;
                    state.dp = compute_dubins_for_leg(waypoints, 1, leg_chi_e(waypoints,1), R);
                end
                % else: final leg — stay in seg 3
            end

        otherwise
            state.dubins_seg = 1;
    end

    % -----------------------------------------------------------------------
    %  Build path struct for current segment
    % -----------------------------------------------------------------------
    dp   = state.dp;
    wp_i = state.wp_idx;
    Va_c = waypoints(min(wp_i+1, N_wp), 4);

    switch state.dubins_seg

        case 1  % Orbit around start circle
            path.type   = 'orbit';
            path.c      = dp.c_s;
            path.rho    = R;
            path.lambda = dp.lambda_s;
            path.Va     = Va_c;

        case 2  % Straight line z1 → z2
            path.type = 'line';
            path.r    = dp.z1;
            path.q    = dp.q;
            path.Va   = Va_c;

        case 3  % Orbit around end circle
            path.type   = 'orbit';
            path.c      = dp.c_e;
            path.rho    = R;
            path.lambda = dp.lambda_e;
            path.Va     = Va_c;

        otherwise
            % Fallback: straight to next waypoint
            path.type = 'line';
            wp_pos = waypoints(min(wp_i+1,N_wp), 1:3)';
            path.r  = waypoints(wp_i, 1:3)';
            leg_v   = wp_pos - path.r;
            lm = norm(leg_v); if lm<1e-6; lm=1; end
            path.q  = leg_v / lm;
            path.Va = Va_c;
    end

end

% =========================================================================
%  Local helpers
% =========================================================================

function dp = compute_dubins_for_leg(waypoints, wp_i, chi_s_in, R)
    % Compute Dubins parameters from waypoint wp_i to wp_i+1.
    % chi_s_in : heading when arriving at wp_i (either aircraft heading
    %            or end heading of previous Dubins path)
    % chi_e    : heading of NEXT leg (wp_{i+1} to wp_{i+2})

    wp_i1  = wp_i + 1;

    ps = waypoints(wp_i,  1:3)';
    pe = waypoints(wp_i1, 1:3)';

    chi_e = leg_chi_e(waypoints, wp_i);

    dp = dubins_parameters(ps, chi_s_in, pe, chi_e, R);
end

% -------------------------------------------------------------------------
function chi_e = leg_chi_e(waypoints, wp_i)
    % Desired end heading for leg wp_i → wp_i+1.
    % Uses direction of NEXT leg (wp_{i+1} → wp_{i+2}).
    % Falls back to current leg direction for the final leg.
    N_wp  = size(waypoints, 1);
    wp_i1 = wp_i + 1;
    if wp_i1 < N_wp
        dv = waypoints(wp_i1+1, 1:2)' - waypoints(wp_i1, 1:2)';
    else
        dv = waypoints(wp_i1, 1:2)' - waypoints(wp_i, 1:2)';
    end
    chi_e = atan2(dv(2), dv(1));
end

% -------------------------------------------------------------------------
function q_out = outgoing_dir(waypoints, wp_next, N_wp)
    % Direction of the outgoing leg starting at wp_next.
    % Book Algorithm 5: seg 3 half-plane normal = q_{i+1} (outgoing direction).
    % This is the direction from wp_next toward wp_next+1.
    % For the final waypoint, use the same leg direction (no next leg).
    if wp_next < N_wp
        q_out = unit2d(waypoints(wp_next+1,1:2)' - waypoints(wp_next,1:2)');
    else
        q_out = unit2d(waypoints(wp_next,1:2)' - waypoints(wp_next-1,1:2)');
    end
end

% -------------------------------------------------------------------------
function u = unit2d(v)
    m = norm(v);
    if m > 1e-6; u = v/m; else; u = [1;0]; end
end
