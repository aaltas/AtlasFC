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
    % genpath adds all subfolders — but ekf_v1/, ekf_v2/ etc. must NOT be
    % added blindly (they contain conflicting function names).
    % ekf_select.m handles versioned EKF path management at runtime.
    core_paths = genpath(fullfile(project_root, 'core'));
    core_paths = remove_ekf_versions(core_paths, project_root);
    addpath(core_paths);

    % Default EKF version for chapter scripts and direct calls
    addpath(fullfile(project_root, 'core', 'estimator', 'ekf_v1'));

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

% -------------------------------------------------------------------------
function filtered = remove_ekf_versions(path_str, project_root)
% Remove ekf_vN subfolders from a genpath string so that versioned EKF
% functions don't conflict.  ekf_select.m adds the correct version at runtime.
    estimator_root = fullfile(project_root, 'core', 'estimator');
    entries  = strsplit(path_str, pathsep);
    keep     = true(1, numel(entries));
    for i = 1:numel(entries)
        e = entries{i};
        % Drop any folder that is a direct child of estimator/ and matches ekf_vN
        if strncmp(e, estimator_root, length(estimator_root))
            rel = e(length(estimator_root)+1:end);
            if ~isempty(regexp(rel, ['^' regexptranslate('escape', filesep) 'ekf_v\d'], 'once'))
                keep(i) = false;
            end
        end
    end
    filtered = strjoin(entries(keep), pathsep);
end
