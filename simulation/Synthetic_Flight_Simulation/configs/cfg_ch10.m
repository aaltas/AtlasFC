% =========================================================================
%  CFG_CH10 - Simulation config: Ch10 Dubins path manager
% =========================================================================
%  Extends cfg_ch09 with Dubins path following enabled.
%  The same rectangular waypoint pattern is flown, but path transitions
%  now use Dubins arc-line-arc segments that respect R_min.
%
%  Key change vs Ch9:
%    use_dubins = true   → path_manager_dubins replaces waypoint_manager
%    R_min               → minimum turning radius [m]
%
%  Minimum turning radius estimate (coordinated turn):
%    R_min = Va² / (g * tan(phi_max)) = 25² / (9.81 * tan(25°)) ≈ 136 m
%  We use R_min = 80 m (slightly aggressive but achievable with the
%  autopilot lateral gains tuned for Ch6).
%
%  Usage:
%    cfg     = cfg_ch10();
%    results = atlas_sim(cfg);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 10
% =========================================================================

function cfg = cfg_ch10()

    % Start from Ch9 (guidance already on, same waypoints)
    cfg = cfg_ch09();

    % -----------------------------------------------------------------------
    %  Enable Dubins path manager
    % -----------------------------------------------------------------------
    cfg.use_dubins      = true;
    cfg.loop_waypoints  = true;   % Continuous laps for valid cross-track metric

    % -----------------------------------------------------------------------
    %  Minimum turning radius [m]
    %  Must satisfy: all inter-waypoint distances > 2 * R_min
    %  400m legs → 2*R_min = 160m  (satisfied for R_min = 80m)
    % -----------------------------------------------------------------------
    cfg.R_min = 120.0;  % [m]  phi≈27.7° at Va=25m/s — autopilot comfortable range
    %  R_min = 80m  → phi≈38.5°  (too steep, causes altitude loss)
    %  R_min = 120m → phi≈27.7°  (manageable)
    %  R_min = 150m → phi≈22.8°  (conservative)

    % -----------------------------------------------------------------------
    %  Simulation time
    %  Dubins arcs add ~pi*R per corner ≈ 250m of arc, so slightly more
    %  time is needed compared to Ch9.
    % -----------------------------------------------------------------------
    cfg.T = 240.0;      % [s]  (Ch9 needed 200s; extend for curved paths)

end
