% =========================================================================
%  SETUP_PATHS - Add all project folders to MATLAB path
% =========================================================================
%  Run this script once when you open the project in MATLAB.
%  It adds core libraries, parameters, tools, and chapter folders.
%
%  Usage:
%    >> setup_paths
% =========================================================================

fprintf('Setting up Small UAV project paths...\n');

project_root = fileparts(mfilename('fullpath'));

addpath(genpath(fullfile(project_root, 'core')));
addpath(genpath(fullfile(project_root, 'params')));
addpath(genpath(fullfile(project_root, 'tools')));
addpath(genpath(fullfile(project_root, 'simulation')));
addpath(genpath(fullfile(project_root, 'tests')));

fprintf('All paths added. Project root: %s\n', project_root);
fprintf('Ready to fly!\n');
