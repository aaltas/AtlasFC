% =========================================================================
%  ATLAS_SIM - AtlasFC reusable simulation runner
% =========================================================================
%  Pure computation engine — no prints, no plots.
%  Takes a config struct, returns a results struct.
%
%  Designed to be called from:
%    • Chapter main scripts  (single run, then plot/analyse there)
%    • Monte Carlo scripts   (many runs, different seeds)
%    • Parameter sweeps      (vary Q, R, gains, etc.)
%
%  PIPELINE (controlled by cfg flags):
%    True dynamics → Sensors (cfg.use_sensors)
%                  → EKF    (cfg.use_ekf)
%                  → Autopilot
%                  → [Guidance — cfg.guidance_on, Ch9+]
%                  → [Planning — cfg.planning_on, Ch10+]
%
%  ADDING A NEW MODULE (e.g. Ch9 guidance):
%    1. Add cfg flag:   cfg.guidance_on = false  in cfg_ch08.m
%    2. Add the module call in SECTION 3 below (guarded by the flag)
%    3. Create cfg_ch09.m with guidance_on = true
%    Existing configs and mains are untouched.
%
%  Usage:
%    cfg     = cfg_ch08();
%    results = atlas_sim(cfg);
%
%  Inputs:
%    cfg  struct  from cfg_chXX.m
%
%  Outputs:
%    results struct with fields:
%      .t          [1×N]   time vector [s]
%      .true        struct  true state histories
%      .est         struct  EKF estimated state histories
%      .sigma       struct  1-sigma uncertainty (from P diagonal)
%      .innov       struct  sensor innovations
%      .cmd         struct  commanded values history
%      .cfg         struct  the config used (for reference)
%
%  Author : AtlasFC  |  Ref: Beard & McLain, Chs. 6-8
% =========================================================================

function results = atlas_sim(cfg)

    % -----------------------------------------------------------------------
    %  Setup
    % -----------------------------------------------------------------------
    rng(cfg.rng_seed);

    % Select EKF version — adds core/estimator/ekf_vN/ to top of path
    ekf_ver    = 'ekf_v1';
    if isfield(cfg, 'ekf_version'), ekf_ver = cfg.ekf_version; end
    atlas_root = fileparts(fileparts(fileparts(which('atlas_sim'))));
    ekf_select(ekf_ver, atlas_root);

    params  = mav_params();
    sparams = sensor_params();
    ep      = ekf_params(sparams);

    % Load trim data
    params_dir = fileparts(which('mav_params'));
    trim_file  = fullfile(params_dir, 'trim_data.mat');
    if ~exist(trim_file, 'file')
        error('trim_data.mat not found. Run ch05_main.m first.');
    end
    load(trim_file, 'x_trim', 'u_trim', 'gains', 'Va_star');  %#ok<LOAD>

    % Build 13-state trim
    e_q_trim = euler_to_quaternion(x_trim(7), x_trim(8), x_trim(9));
    x13_trim = [x_trim(1:6); e_q_trim; x_trim(10:12)];
    x13_trim(1:3) = [0; 0; -cfg.h0];

    % -----------------------------------------------------------------------
    %  Pre-allocate
    % -----------------------------------------------------------------------
    dt = cfg.dt;
    N  = round(cfg.T / dt);

    t_hist = zeros(1, N);

    % True states
    pn_t  = zeros(1,N);  pe_t  = zeros(1,N);  pd_t   = zeros(1,N);
    Va_t = zeros(1,N);  h_t   = zeros(1,N);  chi_t   = zeros(1,N);
    phi_t = zeros(1,N); theta_t = zeros(1,N);
    p_t   = zeros(1,N); q_t   = zeros(1,N);  r_t     = zeros(1,N);

    % Estimated states
    pn_e  = zeros(1,N);  pe_e  = zeros(1,N);  pd_e   = zeros(1,N);
    Va_e  = zeros(1,N); h_e   = zeros(1,N);  chi_e   = zeros(1,N);
    phi_e = zeros(1,N); theta_e = zeros(1,N);
    bp_e  = zeros(1,N); bq_e  = zeros(1,N);  br_e    = zeros(1,N);

    % Uncertainty (1-sigma from P diagonal)
    sig_h   = zeros(1,N); sig_Va  = zeros(1,N);
    sig_phi = zeros(1,N); sig_theta = zeros(1,N); sig_psi = zeros(1,N);

    % Innovations
    innov_baro  = zeros(1,N); innov_pitot = zeros(1,N);
    innov_mag   = zeros(1,N); innov_gps   = zeros(4,N);

    % Commands
    Va_cmd_h  = zeros(1,N); h_cmd_h   = zeros(1,N); chi_cmd_h = zeros(1,N);

    % NEES / NIS  (only allocated when cfg.ekf_analysis = true)
    do_analysis = isfield(cfg,'ekf_analysis') && cfg.ekf_analysis && cfg.use_ekf;
    if do_analysis
        nees_pos  = zeros(1,N);   % NEES position  (df=3, expected=3)
        nees_vel  = zeros(1,N);   % NEES velocity  (df=3, expected=3)
        nis_baro  = NaN(1,N);     % NIS baro       (df=1, expected=1)
        nis_pitot = NaN(1,N);     % NIS pitot      (df=1, expected=1)
        nis_mag   = NaN(1,N);     % NIS mag        (df=1, expected=1)
        nis_gps   = NaN(1,N);     % NIS GPS        (df=4, expected=4)
    end

    % -----------------------------------------------------------------------
    %  Initialize
    % -----------------------------------------------------------------------
    x13 = x13_trim;

    if cfg.use_sensors
        sensor_state = sensors_init(sparams, x13_trim);
    end

    if cfg.use_ekf
        [x_hat, P] = ekf_init(x13_trim, ep);
        % Inject initial estimation error
        x_hat(1:3) = x_hat(1:3) + cfg.ekf_pos_err * randn(3,1);
        x_hat(4:6) = x_hat(4:6) + cfg.ekf_vel_err * randn(3,1);
    else
        x_hat = [];  P = [];
    end

    ap_state.Va_int = 0;

    if cfg.guidance_on
        guidance_state = guidance_init(cfg.waypoints, cfg);
    end

    % -----------------------------------------------------------------------
    %  SECTION 3 — Simulation loop
    % -----------------------------------------------------------------------
    for k = 1:N
        t_now     = (k-1) * dt;
        t_hist(k) = t_now;

        % --- Command schedule ---
        cmd.Va_c  = cfg.Va_init  + (cfg.Va_step  - cfg.Va_init)  * (t_now >= cfg.t_Va);
        cmd.h_c   = cfg.h_init   + (cfg.h_step   - cfg.h_init)   * (t_now >= cfg.t_h);
        cmd.chi_c = cfg.chi_init + (cfg.chi_step - cfg.chi_init)  * (t_now >= cfg.t_chi);

        Va_cmd_h(k)  = cmd.Va_c;
        h_cmd_h(k)   = cmd.h_c;
        chi_cmd_h(k) = cmd.chi_c;

        % --- True state observables ---
        R_true = quaternion_to_rotation(x13(7:10));
        [phi_true, theta_true, psi_true] = rotation_to_euler(R_true);
        pn_t(k)    = x13(1);
        pe_t(k)    = x13(2);
        pd_t(k)    = x13(3);
        Va_t(k)    = sqrt(x13(4)^2 + x13(5)^2 + x13(6)^2);
        h_t(k)     = -x13(3);
        chi_t(k)   = psi_true;
        phi_t(k)   = phi_true;
        theta_t(k) = theta_true;
        p_t(k)     = x13(11);
        q_t(k)     = x13(12);
        r_t(k)     = x13(13);

        % --- Forces at true state ---
        [fm, ~, ~, ~] = forces_moments(x13, u_trim, zeros(3,1), zeros(3,1), params);

        % --- Sensors ---
        if cfg.use_sensors
            [y, sensor_state] = sensors(x13, fm, sensor_state, sparams, params, dt);
        else
            % Perfect sensors — true state directly
            y.gyro  = x13(11:13);
            y.accel = fm(1:3)/params.mass - R_true*[0;0;params.gravity];
            y.baro  = h_t(k);        y.baro_new  = true;
            y.pitot = Va_t(k);       y.pitot_new = true;
            y.mag   = psi_true;      y.mag_new   = true;
            y.gps   = [x13(1:2); Va_t(k); psi_true];  y.gps_new = true;
        end

        % --- EKF ---
        if cfg.use_ekf
            [x_hat, P, innov] = ekf(x_hat, P, y, ep, params, dt);

            R_est = quaternion_to_rotation(x_hat(7:10));
            [phi_est, theta_est, psi_est] = rotation_to_euler(R_est);

            pn_e(k)    = x_hat(1);
            pe_e(k)    = x_hat(2);
            pd_e(k)    = x_hat(3);
            Va_e(k)    = sqrt(x_hat(4)^2 + x_hat(5)^2 + x_hat(6)^2);
            h_e(k)     = -x_hat(3);
            chi_e(k)   = psi_est;
            phi_e(k)   = phi_est;
            theta_e(k) = theta_est;
            bp_e(k)    = x_hat(11);
            bq_e(k)    = x_hat(12);
            br_e(k)    = x_hat(13);

            sig_h(k)     = sqrt(P(3,3));
            sig_Va(k)    = sqrt((P(4,4)+P(5,5)+P(6,6))/3);
            sig_phi(k)   = sqrt(P(8,8)+P(9,9))*2;
            sig_theta(k) = sqrt(P(8,8)+P(9,9))*2;
            sig_psi(k)   = sqrt(P(9,9)+P(10,10))*2;

            innov_baro(k)   = innov.baro;
            innov_pitot(k)  = innov.pitot;
            innov_mag(k)    = innov.mag;
            innov_gps(:,k)  = innov.gps;

            % --- NEES / NIS (only when ekf_analysis flag is set) ---
            if do_analysis
                % NEES — position (indices 1:3)
                e_pos = x13(1:3) - x_hat(1:3);
                nees_pos(k) = e_pos' / P(1:3,1:3) * e_pos;

                % NEES — velocity (indices 4:6)
                e_vel = x13(4:6) - x_hat(4:6);
                nees_vel(k) = e_vel' / P(4:6,4:6) * e_vel;

                % NIS — approximate: ν²/R  (lower bound; exact uses S=HPH'+R)
                % Valid approximation when P diagonal << R (steady-state filter)
                if y.baro_new
                    nis_baro(k)  = innov.baro^2  / ep.R_baro;
                end
                if y.pitot_new
                    nis_pitot(k) = innov.pitot^2 / ep.R_pitot;
                end
                if y.mag_new
                    nis_mag(k)   = innov.mag^2   / ep.R_mag;
                end
                if y.gps_new
                    nis_gps(k)   = innov.gps' / ep.R_gps * innov.gps;
                end
            end

            % Autopilot input: EKF estimate
            x12_ap = [x_hat(1:6); phi_est; theta_est; psi_est; x_hat(11:13)];
            x12_ap(10:12) = y.gyro - x_hat(11:13);
        else
            % Autopilot input: true state (Ch6 mode)
            [phi_ap, theta_ap, psi_ap] = rotation_to_euler(R_true);
            x12_ap = [x13(1:6); phi_ap; theta_ap; psi_ap; x13(11:13)];
            % Copy true → estimated for results
            pn_e(k) = pn_t(k); pe_e(k) = pe_t(k); pd_e(k) = pd_t(k);
            Va_e(k) = Va_t(k); h_e(k) = h_t(k); chi_e(k) = chi_t(k);
            phi_e(k) = phi_t(k); theta_e(k) = theta_t(k);
        end

        % --- Guidance (Ch9+) — overrides cmd.chi_c and cmd.h_c ---
        %     Must come AFTER x12_ap is built (EKF or true-state block above)
        if cfg.guidance_on
            pos = x12_ap(1:3);
            chi = x12_ap(9);   % psi ≈ course (small sideslip assumption)

            % Select path manager: Dubins (Ch10), Fillet (Ch11), or straight-line (Ch9)
            if isfield(cfg, 'use_dubins') && cfg.use_dubins
                [path, guidance_state] = path_manager_dubins( ...
                    cfg.waypoints, pos, chi, guidance_state, cfg);
            elseif isfield(cfg, 'use_fillet') && cfg.use_fillet
                [path, guidance_state] = path_manager_fillet( ...
                    cfg.waypoints, pos, guidance_state, cfg);
            else
                [path, guidance_state] = waypoint_manager( ...
                    cfg.waypoints, pos, guidance_state, cfg);
            end

            if strcmp(path.type, 'line')
                [cmd.chi_c, cmd.h_c, ~] = follow_straight_line(path, pos, chi);
            elseif strcmp(path.type, 'orbit')
                [cmd.chi_c, cmd.h_c, ~] = follow_orbit(path, pos, chi);
            end
            cmd.Va_c = path.Va;
        end

        % --- Autopilot ---
        [u_cmd, ap_state] = autopilot(cmd, x12_ap, u_trim, gains, ap_state, dt);

        % --- Propagate true dynamics ---
        [fm_true, ~, ~, ~] = forces_moments(x13, u_cmd, zeros(3,1), zeros(3,1), params);
        x13 = mav_dynamics(x13, fm_true, params, dt);
    end

    % -----------------------------------------------------------------------
    %  Pack results
    % -----------------------------------------------------------------------
    results.t   = t_hist;
    results.cfg = cfg;

    results.true.pn    = pn_t;
    results.true.pe    = pe_t;
    results.true.pd    = pd_t;
    results.true.Va    = Va_t;
    results.true.h     = h_t;
    results.true.chi   = chi_t;
    results.true.phi   = phi_t;
    results.true.theta = theta_t;
    results.true.p     = p_t;
    results.true.q     = q_t;
    results.true.r     = r_t;

    results.est.pn    = pn_e;
    results.est.pe    = pe_e;
    results.est.pd    = pd_e;
    results.est.Va    = Va_e;
    results.est.h     = h_e;
    results.est.chi   = chi_e;
    results.est.phi   = phi_e;
    results.est.theta = theta_e;
    results.est.bp    = bp_e;
    results.est.bq    = bq_e;
    results.est.br    = br_e;

    results.sigma.h     = sig_h;
    results.sigma.Va    = sig_Va;
    results.sigma.phi   = sig_phi;
    results.sigma.theta = sig_theta;
    results.sigma.psi   = sig_psi;

    results.innov.baro  = innov_baro;
    results.innov.pitot = innov_pitot;
    results.innov.mag   = innov_mag;
    results.innov.gps   = innov_gps;

    results.cmd.Va  = Va_cmd_h;
    results.cmd.h   = h_cmd_h;
    results.cmd.chi = chi_cmd_h;

    % NEES / NIS — only populated when cfg.ekf_analysis = true
    if do_analysis
        results.nees.pos  = nees_pos;
        results.nees.vel  = nees_vel;
        results.nis.baro  = nis_baro;
        results.nis.pitot = nis_pitot;
        results.nis.mag   = nis_mag;
        results.nis.gps   = nis_gps;
    end

    % NOTE: guidance_state is not saved to results (it is internal to
    % the guidance module). Guidance errors and path tracking performance
    % should be computed in chapter main scripts using results.true and
    % results.est trajectory data along with cfg.waypoints.

end
