% =========================================================================
%  CFG_CH09 - Simulation config: Ch9 path following guidance
% =========================================================================
%  Extends cfg_ch08 with waypoint-based guidance enabled.
%  Aircraft follows a rectangular waypoint loop at 100m altitude.
%
%  Waypoint path:
%    WP1: (0,    0,   -100) m
%    WP2: (400,  0,   -100) m
%    WP3: (400, 400,  -100) m
%    WP4: (0,   400,  -100) m
%    WP5: (0,    0,   -100) m (back to start)
%
%  Desired airspeed: 25 m/s throughout
%  Simulation time: 200 s
%
%  Usage:
%    cfg     = cfg_ch09();
%    results = atlas_sim(cfg);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 9
% =========================================================================

function cfg = cfg_ch09()

    % Start from Ch8 baseline and enable guidance
    cfg = cfg_ch08();

    % -----------------------------------------------------------------------
    %  Enable guidance module (Ch9)
    % -----------------------------------------------------------------------
    cfg.guidance_on = true;

    % -----------------------------------------------------------------------
    %  Simulation timing (extend for path following)
    % -----------------------------------------------------------------------
    cfg.T = 200.0;    % [s]  total duration (200s for rectangular loop)

    % -----------------------------------------------------------------------
    %  Waypoint definition [pn, pe, pd, Va_c]
    %  Forms a 400×400 m rectangle at 100m altitude
    % -----------------------------------------------------------------------
    cfg.waypoints = [
        0,    0, -100,  25;      % WP1: Southwest corner
        400,  0, -100,  25;      % WP2: Southeast corner
        400, 400, -100,  25;      % WP3: Northeast corner
        0,   400, -100,  25;      % WP4: Northwest corner
        0,    0, -100,  25;      % WP5: Back to start
    ];

    cfg.loop_waypoints = true;    % Loop rectangle continuously (needed for valid cross-track metric)

    % -----------------------------------------------------------------------
    %  Command schedule (guidance overrides these)
    % -----------------------------------------------------------------------
    % These are baseline commands; guidance overrides chi_c and h_c
    cfg.Va_init  = 25.0;          % Airspeed: constant at 25 m/s
    cfg.Va_step  = 25.0;          % No airspeed change
    cfg.t_Va     = 1000.0;        % (never reached)

    cfg.h_init   = 100.0;         % Altitude: 100m (guidance controls)
    cfg.h_step   = 100.0;         % No altitude change
    cfg.t_h      = 1000.0;        % (never reached)

    cfg.chi_init = 0.0;           % Course: 0° (guidance controls)
    cfg.chi_step = 0.0;           % No course change
    cfg.t_chi    = 1000.0;        % (never reached)

    % -----------------------------------------------------------------------
    %  Initial conditions
    % -----------------------------------------------------------------------
    cfg.h0      = 100.0;          % Start at 100m altitude
    cfg.Va_init = 25.0;           % Start at 25 m/s

end
