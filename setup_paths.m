% =========================================================================
%  SETUP_PATHS - Add all AtlasFC project folders to MATLAB path
% =========================================================================
%  Can be called from any script using:
%    run(fullfile(atlas_root, 'setup_paths.m'))
%  Or run manually from the AtlasFC root:
%    >> setup_paths
% =========================================================================

project_root = fileparts(mfilename('fullpath'));

% Core library modules (the reusable flight controller building blocks)
addpath(genpath(fullfile(project_root, 'core')));

% Aircraft and simulation parameters
addpath(genpath(fullfile(project_root, 'params')));

% Shared plotting and analysis utilities
addpath(genpath(fullfile(project_root, 'tools')));

% Closed-loop simulation harness
addpath(genpath(fullfile(project_root, 'simulation')));

% Unit tests
addpath(genpath(fullfile(project_root, 'tests')));

% Chapter study scripts (so chapter viewers can call each other if needed)
addpath(genpath(fullfile(project_root, 'chapters')));

fprintf('[AtlasFC] Paths loaded. Root: %s\n', project_root);
