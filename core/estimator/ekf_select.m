% =========================================================================
%  EKF_SELECT  —  EKF version selector
% =========================================================================
%  Adds the requested EKF version folder to the MATLAB path so that
%  calls to ekf(), ekf_params(), ekf_init() etc. resolve to that version.
%
%  Call this ONCE at the start of atlas_sim.m and run_flight_eval.m,
%  before any EKF function is invoked.
%
%  Usage:
%    ekf_select(cfg.ekf_version, atlas_root)
%
%  Inputs:
%    version     string  'ekf_v1' | 'ekf_v2' | ...
%    atlas_root  string  absolute path to AtlasFC root
%
%  Adding a new EKF version:
%    1. Create core/estimator/ekf_vN/ folder
%    2. Implement ekf.m, ekf_init.m, ekf_params.m, ekf_propagate.m,
%       ekf_update.m  (same function signatures as ekf_v1)
%    3. Set cfg.ekf_version = 'ekf_vN' in your config
%    No other files need to change.
%
%  Author : AtlasFC
% =========================================================================

function ekf_select(version, atlas_root)

    ekf_dir = fullfile(atlas_root, 'core', 'estimator', version);

    if ~exist(ekf_dir, 'dir')
        error('EKF version ''%s'' not found.\n  Expected: %s\n', version, ekf_dir);
    end

    % Add version folder at the TOP of the path so it takes priority
    % over the base estimator folder (which keeps the shared nav_equations,
    % skew3, etc. as fallback).
    addpath(ekf_dir, '-begin');

    fprintf('  [ekf_select] Using %s  (%s)\n', version, ekf_dir);
end
