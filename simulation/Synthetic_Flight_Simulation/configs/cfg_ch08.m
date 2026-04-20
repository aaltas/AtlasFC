% =========================================================================
%  CFG_CH08 - Simulation config: Ch8 full pipeline (V1 baseline)
% =========================================================================
%  Covers: dynamics → sensors → EKF → autopilot
%  No guidance, no planning — closed-loop attitude/altitude/course hold.
%
%  Usage:
%    cfg     = cfg_ch08();
%    results = atlas_sim(cfg);
%
%  To create a new config (e.g. Ch9 + guidance):
%    cfg = cfg_ch08();          % start from Ch8 baseline
%    cfg.guidance_on = true;    % enable new module
%    cfg.T = 120;               % extend duration
%    results = atlas_sim(cfg);
%
%  Author : AtlasFC
% =========================================================================

function cfg = cfg_ch08()

    % -----------------------------------------------------------------------
    %  Simulation timing
    % -----------------------------------------------------------------------
    cfg.dt       = 0.01;    % [s]  time step (IMU rate = 100 Hz)
    cfg.T        = 60.0;    % [s]  total duration
    cfg.rng_seed = 0;       % fixed seed → reproducible noise

    % -----------------------------------------------------------------------
    %  Initial flight condition
    % -----------------------------------------------------------------------
    cfg.Va0 = 25.0;         % [m/s]  initial airspeed (= trim)
    cfg.h0  = 100.0;        % [m]    initial altitude

    % -----------------------------------------------------------------------
    %  Command schedule (step inputs)
    % -----------------------------------------------------------------------
    cfg.Va_init  = 25.0;          cfg.Va_step  = 28.0;         cfg.t_Va  = 5.0;
    cfg.h_init   = 100.0;         cfg.h_step   = 130.0;        cfg.t_h   = 25.0;
    cfg.chi_init = 0.0;           cfg.chi_step = deg2rad(45);  cfg.t_chi = 45.0;

    % -----------------------------------------------------------------------
    %  EKF initial perturbation (inject small error to test convergence)
    % -----------------------------------------------------------------------
    cfg.ekf_pos_err = 2.0;   % [m]    ±σ added to initial position estimate
    cfg.ekf_vel_err = 0.3;   % [m/s]  ±σ added to initial velocity estimate

    % -----------------------------------------------------------------------
    %  Active modules
    %  (add new flags here as chapters progress)
    % -----------------------------------------------------------------------
    cfg.use_sensors  = true;   % Ch7: sensor noise model
    cfg.use_ekf      = true;   % Ch8: EKF state estimator
    cfg.guidance_on  = false;  % Ch9: path following guidance  (not yet)
    cfg.planning_on  = false;  % Ch10: path planning           (not yet)

    % -----------------------------------------------------------------------
    %  EKF version  — switch here to test a new estimator design.
    %  Everything else (sim loop, autopilot, plots) stays unchanged.
    % -----------------------------------------------------------------------
    cfg.ekf_version = 'ekf_v1';   % 'ekf_v1' | 'ekf_v2' | ...

    % -----------------------------------------------------------------------
    %  Analysis flags  (off by default — set true in analysis scripts)
    % -----------------------------------------------------------------------
    cfg.ekf_analysis = false;  % enable NEES/NIS computation in atlas_sim

end
