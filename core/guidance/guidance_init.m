% =========================================================================
%  GUIDANCE_INIT - Initialize guidance module state
% =========================================================================
%  Allocates and initializes the guidance state struct for the straight-line
%  waypoint manager (Ch9), fillet path manager (Ch11), and Dubins path
%  manager (Ch10).
%
%  INPUTS:
%    waypoints [N×4]    Waypoint matrix [pn, pe, pd, Va_c]
%    cfg       [struct] Configuration with guidance parameters.
%                       cfg.use_dubins = true  → allocates Dubins state.
%                       cfg.use_fillet = true  → allocates fillet state.
%
%  OUTPUTS:
%    guidance_state [struct]  State struct:
%        .wp_idx           [1×1]  Current waypoint leg index (1-based)
%        .dubins_seg       [1×1]  Dubins segment (1=arc1, 2=line, 3=arc2)  [Dubins only]
%        .dp               [struct] Active Dubins path parameters            [Dubins only]
%        .dubins_arc1_neg  [bool] Latch: UAV has been behind z1 half-plane  [Dubins only]
%        .fillet_seg       [1×1]  Fillet segment (1=line, 2=orbit)          [Fillet only]
%
%  REFERENCE:
%    Beard, R.W. & McLain, T.W. (2012). Small Unmanned Aircraft: Theory
%    and Practice. Princeton University Press. Chapters 9-11.
%
%  Author : AtlasFC
% =========================================================================

function guidance_state = guidance_init(waypoints, cfg)  %#ok<INUSL>

    % Common state
    guidance_state.wp_idx = 1;

    % Dubins path manager additional state (Ch10)
    if isfield(cfg, 'use_dubins') && cfg.use_dubins
        guidance_state.dubins_seg      = 1;     % Start in arc-1 segment
        guidance_state.dp              = [];    % Computed on first call
        guidance_state.dubins_arc1_neg = false; % Arc-1 latch (prevents false seg 1→2 trigger)
    end

    % Fillet path manager additional state (Ch11)
    if isfield(cfg, 'use_fillet') && cfg.use_fillet
        guidance_state.fillet_seg = 1;   % Start in straight-line phase
    end

end
