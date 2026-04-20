% =========================================================================
%  ANALYZE_CFG  —  Analyze_Simulation configuration
% =========================================================================
%  Single control point for all analysis scripts under Analyze_Simulation/.
%  Each analysis script reads this config — change here, affects all.
%
%  Usage:
%    cfg = analyze_cfg();
%
%  Author : AtlasFC
% =========================================================================

function cfg = analyze_cfg()

    % -----------------------------------------------------------------------
    %  Base simulation config
    %  The analysis runs atlas_sim internally — this selects which chapter
    %  config to use as the foundation.
    % -----------------------------------------------------------------------
    cfg.sim_config = 'cfg_ch08';    % function name: 'cfg_ch08' | 'cfg_ch09' | ...

    % -----------------------------------------------------------------------
    %  EKF version
    %  Change here to run all analyses with a different estimator.
    %  'ekf_v1' = current baseline  |  'ekf_v2' = new design
    % -----------------------------------------------------------------------
    cfg.ekf_version = 'ekf_v1';

    % -----------------------------------------------------------------------
    %  Monte Carlo settings  (used by run_ekf_analysis.m)
    % -----------------------------------------------------------------------
    cfg.N_MC         = 50;      % number of independent runs
    cfg.mc_seed_base = 1;       % seeds: mc_seed_base : mc_seed_base + N_MC - 1

    % -----------------------------------------------------------------------
    %  NEES / NIS analysis  (passed to atlas_sim via cfg.ekf_analysis)
    % -----------------------------------------------------------------------
    cfg.ekf_analysis = true;    % always true for Analyze_EKF

    % -----------------------------------------------------------------------
    %  Chi-squared confidence level
    % -----------------------------------------------------------------------
    cfg.confidence = 0.95;      % 95% bounds on NEES / NIS plots

    % -----------------------------------------------------------------------
    %  RMSE burn-in  (exclude filter convergence transient)
    % -----------------------------------------------------------------------
    cfg.rmse_burn_in = 5.0;     % [s]

end
