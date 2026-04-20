% =========================================================================
%  CFG_CH11 - Simulation config: Ch11 Fillet Path Manager
% =========================================================================
%  Extends cfg_ch09 with the fillet path manager enabled.
%  The same 400×400 m rectangular waypoint loop is flown, but waypoint
%  transitions now use smooth circular fillet arcs instead of sharp corners.
%
%  Key change vs Ch9:
%    use_fillet = true  → path_manager_fillet replaces waypoint_manager
%    R_fillet          → fillet arc radius [m]
%
%  Fillet radius choice:
%    R_fillet = 80 m  → phi ≈ 38° at Va=25 m/s  (aggressive but visible)
%    R_fillet = 60 m  → phi ≈ 31° at Va=25 m/s  (comfortable)
%    We use 80 m for clearly visible rounded corners in the plot.
%
%  Usage:
%    cfg     = cfg_ch11();
%    results = atlas_sim(cfg);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 11
% =========================================================================

function cfg = cfg_ch11()

    % Start from Ch9 (guidance on, same rectangular waypoints)
    cfg = cfg_ch09();

    % -----------------------------------------------------------------------
    %  Enable fillet path manager
    % -----------------------------------------------------------------------
    cfg.use_fillet      = true;
    cfg.use_dubins      = false;   % Explicit: not Dubins
    cfg.loop_waypoints  = true;    % Continuous laps for valid cross-track metric

    % -----------------------------------------------------------------------
    %  Fillet radius [m]
    %  Waypoint legs are 400 m — fillet distance d = R/tan(45°) = 80 m
    %  so the fillet starts 80 m before each corner → well within the leg.
    % -----------------------------------------------------------------------
    cfg.R_fillet = 80.0;   % [m]

    % -----------------------------------------------------------------------
    %  Simulation time — fillet adds slightly more arc than sharp corners
    % -----------------------------------------------------------------------
    cfg.T = 220.0;   % [s]  (Ch9 needed 200 s; small margin for arcs)

end
