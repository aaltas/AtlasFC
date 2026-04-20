% =========================================================================
%  RRT_PLAN - Rapidly-exploring Random Tree path planner
% =========================================================================
%  Plans a collision-free 2-D (constant altitude) path from start to goal
%  using the RRT algorithm (Beard & McLain, Ch.11).
%
%  The tree is grown in the horizontal plane; altitude is held constant at
%  the start altitude throughout (flat-earth assumption for Ch11).
%
%  ALGORITHM:
%    1. Initialize tree with root at q_start
%    2. For each iteration:
%       a. Sample q_rand from map (with goal-bias probability p_goal)
%       b. Find nearest node q_near in current tree
%       c. Steer from q_near toward q_rand by step_size → q_new
%       d. Check collision_check(q_near, q_new, obstacles)
%       e. If clear: add q_new to tree with parent q_near
%       f. If ||q_new - q_goal||_2D < goal_radius: success → extract path
%    3. Smooth extracted path with smooth_path
%    4. Return waypoint list
%
%  INPUTS:
%    plan_cfg  [struct]  Planning configuration:
%       .start     [3×1]  Start position [pn, pe, pd]
%       .goal      [3×1]  Goal  position [pn, pe, pd]
%       .step      [1×1]  RRT step size  [m]      (default 200)
%       .max_iter  [1×1]  Max iterations          (default 3000)
%       .goal_rad  [1×1]  Goal acceptance radius  (default step)
%       .p_goal    [1×1]  Goal-bias probability   (default 0.05)
%       .Va        [1×1]  Airspeed command         [m/s]
%    map_cfg   [struct]  Map configuration:
%       .size_n    [1×1]  North extent [m]
%       .size_e    [1×1]  East extent  [m]
%       .obstacles [M×4]  [cn, ce, radius, height] per row
%
%  OUTPUTS:
%    waypoints [N×4]    Planned waypoint matrix [pn, pe, pd, Va_c]
%                       (smoothed; includes start and goal)
%    tree      [struct] Full RRT tree for visualization:
%       .nodes  [K×3]   All sampled nodes [pn, pe, pd]
%       .parent [K×1]   Parent index for each node (0 = root)
%    success   [bool]   true if goal was reached
%
%  REFERENCE:
%    Beard, R.W. & McLain, T.W. (2012). Small Unmanned Aircraft: Theory
%    and Practice. Princeton University Press. Chapter 11.
%
%  Author : AtlasFC
% =========================================================================

function [waypoints, tree, success] = rrt_plan(plan_cfg, map_cfg)

    % --- Parameters ---
    step     = plan_cfg.step;
    max_iter = plan_cfg.max_iter;
    goal_rad = isfield_default(plan_cfg, 'goal_rad', step);
    p_goal   = isfield_default(plan_cfg, 'p_goal',   0.05);
    Va       = isfield_default(plan_cfg, 'Va',        25.0);

    start    = plan_cfg.start(:);   % [3×1]
    goal     = plan_cfg.goal(:);    % [3×1]

    obstacles = map_cfg.obstacles;  % [M×4]
    size_n    = map_cfg.size_n;
    size_e    = map_cfg.size_e;

    % Altitude is constant (flat-earth, held at start altitude)
    pd_const = start(3);

    % --- Initialize tree ---
    nodes  = zeros(max_iter+1, 3);
    parent = zeros(max_iter+1, 1);
    nodes(1, :) = start';
    N_nodes = 1;

    success    = false;
    goal_node  = 0;

    % --- RRT main loop ---
    for iter = 1:max_iter

        % --- Sample ---
        if rand() < p_goal
            % Goal bias
            q_rand = goal;
        else
            % Uniform random sample in map bounds
            q_rand = [rand()*size_n; rand()*size_e; pd_const];
        end

        % --- Nearest node ---
        diffs  = nodes(1:N_nodes, 1:2) - repmat(q_rand(1:2)', N_nodes, 1);
        dists  = sqrt(sum(diffs.^2, 2));
        [~, idx_near] = min(dists);
        q_near = nodes(idx_near, :)';

        % --- Steer ---
        dir_vec = q_rand(1:2) - q_near(1:2);
        d       = norm(dir_vec);
        if d < 1e-6
            continue;
        end
        dir_unit = dir_vec / d;
        q_new    = [q_near(1:2) + min(d, step) * dir_unit; pd_const];

        % --- Collision check ---
        if ~collision_check(q_near, q_new, obstacles)
            continue;   % Segment hits an obstacle — skip
        end

        % --- Add node ---
        N_nodes = N_nodes + 1;
        nodes(N_nodes, :)  = q_new';
        parent(N_nodes)    = idx_near;

        % --- Goal check ---
        dist_to_goal = norm(q_new(1:2) - goal(1:2));
        if dist_to_goal < goal_rad
            % Connect to goal directly if clear
            if collision_check(q_new, goal, obstacles)
                N_nodes = N_nodes + 1;
                nodes(N_nodes, :) = goal';
                parent(N_nodes)   = N_nodes - 1;
                goal_node = N_nodes;
                success   = true;
                break;
            end
        end
    end

    % --- Pack tree struct ---
    tree.nodes  = nodes(1:N_nodes, :);
    tree.parent = parent(1:N_nodes);

    % --- Extract path ---
    if ~success
        warning('RRT_PLAN: goal not reached after %d iterations.', max_iter);
        % Return direct path (may collide) as fallback
        raw_path = [start'; goal'];
    else
        % Trace back from goal to root
        path_idx = goal_node;
        raw_idx  = [];
        while path_idx > 0
            raw_idx  = [path_idx, raw_idx];       %#ok<AGROW>
            path_idx = parent(path_idx);
        end
        raw_path = nodes(raw_idx, :);   % [K×3]
    end

    % --- Smooth path ---
    smooth = smooth_path(raw_path, obstacles);

    % --- Pack waypoints [pn, pe, pd, Va_c] ---
    N_wp = size(smooth, 1);
    waypoints = [smooth, Va * ones(N_wp, 1)];

end

% -------------------------------------------------------------------------
%  Helper: get field with default
% -------------------------------------------------------------------------
function val = isfield_default(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end
