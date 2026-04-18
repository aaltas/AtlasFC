% =========================================================================
%  SETUP_PATHS - Add all AtlasFC project folders to MATLAB path
% =========================================================================
%  Called from chapter scripts with the root path as argument:
%    setup_paths(atlas_root)
%
%  Or run manually from the AtlasFC root directory:
%    >> setup_paths(pwd)
%
%  NOTE: Accepts root as argument to avoid mfilename() temp-path bug
%  on Mac when MATLAB Editor uses a temporary file copy.
% =========================================================================

function setup_paths(project_root)

    if nargin < 1
        % Fallback: called directly from AtlasFC root
        project_root = pwd;
    end

    % Resolve to absolute path
    project_root = char(java.io.File(project_root).getCanonicalPath());

    % Core library modules
    addpath(genpath(fullfile(project_root, 'core')));

    % Aircraft and simulation parameters
    addpath(genpath(fullfile(project_root, 'params')));

    % Shared utilities
    addpath(genpath(fullfile(project_root, 'tools')));

    % Simulation harness
    addpath(genpath(fullfile(project_root, 'simulation')));

    % Unit tests
    addpath(genpath(fullfile(project_root, 'tests')));

    % Chapter scripts
    addpath(genpath(fullfile(project_root, 'chapters')));

    fprintf('[AtlasFC] Paths loaded. Root: %s\n', project_root);

end
