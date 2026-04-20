% =========================================================================
%  CFG_CH12 - Simulation config: Ch12 RRT path planning
% =========================================================================
%  Configures a path planning scenario with cylinder obstacles.
%  The RRT planner is called in ch12_main.m to produce a collision-free
%  waypoint list, which is then flown by the Ch9 straight-line guidance.
%
%  MAP (2000 × 2000 m):
%    Start : (0,    0,    -100 m)
%    Goal  : (1500, 1500, -100 m)
%    8 cylinder obstacles of varying size scattered through the map
%
%  RRT PARAMETERS:
%    step     = 200 m   — maximum branch length per iteration
%    max_iter = 3000    — upper bound on tree expansions
%    goal_rad = 200 m   — acceptance radius around goal
%    p_goal   = 0.08    — probability of biasing sample toward goal
%
%  FLIGHT:
%    Path manager is selectable via cfg flags:
%      use_fillet=false, use_dubins=false → straight-line (Ch9, default)
%      use_fillet=true                    → fillet arcs (Ch11, smooth corners)
%      use_dubins=true                    → Dubins CSC (Ch10, heading-aware)
%    Simulation time is set long enough to fly the full path at 25 m/s.
%
%  Usage:
%    cfg       = cfg_ch12();
%    [wps, tree, ok] = rrt_plan(cfg.plan, cfg.map);
%    cfg.waypoints   = wps;
%    results   = atlas_sim(cfg);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 12
% =========================================================================

function cfg = cfg_ch12()

    % Start from Ch9 baseline (guidance on, sensors on, EKF on)
    cfg = cfg_ch09();

    % -----------------------------------------------------------------------
    %  Map definition
    %  Obstacle format: [center_n, center_e, radius_m, height_m_AGL]
    %  Height is in positive-up metres above ground.
    %  UAV flies at h = 100 m; obstacles below 100 m don't block horizontally
    %  but the collision_check compares h_flight vs obstacle height.
    %  We set all obstacle heights to 150 m so they block the 100 m UAV.
    % -----------------------------------------------------------------------
    cfg.map.size_n    = 2000;   % Map north extent [m]
    cfg.map.size_e    = 2000;   % Map east  extent [m]
    cfg.map.size_d    = 300;    % Map altitude range [m]

    %                cn    ce   radius  height
    cfg.map.obstacles = [
        300,  250,  120,  150;   % OBS-1  — wide cylinder, SW
        600,  700,   90,  150;   % OBS-2  — medium, centre-SW
        750,  300,   80,  150;   % OBS-3  — medium, south
        900,  950,  110,  150;   % OBS-4  — wide, centre
       1100,  500,   75,  150;   % OBS-5  — narrow, mid-east
       1200,  800,  100,  150;   % OBS-6  — medium, east
        500, 1200,   85,  150;   % OBS-7  — medium, north-west
       1300, 1100,   70,  150;   % OBS-8  — narrow, near goal
    ];

    % -----------------------------------------------------------------------
    %  RRT planning parameters
    % -----------------------------------------------------------------------
    cfg.plan.start    = [0;    0;    -100];   % [pn; pe; pd]  (h=100m)
    cfg.plan.goal     = [1500; 1500; -100];   % [pn; pe; pd]
    cfg.plan.step     = 200;      % [m]   max branch length
    cfg.plan.max_iter = 3000;     % [-]   iteration limit
    cfg.plan.goal_rad = 200;      % [m]   goal acceptance radius
    cfg.plan.p_goal   = 0.08;     % [-]   goal-bias probability
    cfg.plan.Va       = 25.0;     % [m/s] airspeed along planned path

    % -----------------------------------------------------------------------
    %  Guidance: path manager selection
    %    use_fillet = false → straight-line (Ch9, sharp corners)
    %    use_fillet = true  → fillet arcs (Ch11, smooth corners)
    %    use_dubins = true  → Dubins CSC (Ch10, heading-constrained)
    % -----------------------------------------------------------------------
    cfg.use_fillet      = false;   % set true to use smooth fillet transitions
    cfg.use_dubins      = false;
    cfg.R_fillet        = 80.0;    % [m]  fillet radius (used if use_fillet=true)
    cfg.R_min           = 120.0;   % [m]  Dubins R_min  (used if use_dubins=true)
    cfg.loop_waypoints  = false;

    % -----------------------------------------------------------------------
    %  Simulation time
    %  Planned path ~2200-2500 m at 25 m/s → ~90-100 s to goal.
    %  150 s gives ~50 s margin for initial transient + convergence.
    %  (300 s was too long: UAV overshoots goal and flies off indefinitely)
    % -----------------------------------------------------------------------
    cfg.T = 150.0;   % [s]

    % -----------------------------------------------------------------------
    %  Initial position: start at planning start point
    % -----------------------------------------------------------------------
    cfg.h0 = 100.0;   % [m]  starting altitude

    % -----------------------------------------------------------------------
    %  Waypoints: placeholder — ch12_main.m replaces this with RRT output
    % -----------------------------------------------------------------------
    cfg.waypoints = [
        0,    0, -100, 25;
        1500, 1500, -100, 25;
    ];

end
