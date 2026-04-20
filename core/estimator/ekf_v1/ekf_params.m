% =========================================================================
%  EKF_PARAMS - Q, R, and P0 matrices for the 13-state INS EKF
% =========================================================================
%  Returns tuning matrices for the EKF.  All values are expressed as
%  DISCRETE variances (per dt step where relevant).
%
%  PROCESS NOISE Q (13×13 diagonal):
%  ─────────────────────────────────
%  Position  (1:3)  : driven by velocity integration — very small
%  Velocity  (4:6)  : driven by accel noise  σ_a²
%  Quaternion(7:10) : driven by gyro noise   (σ_g/2)²
%  Gyro bias (11:13): driven by bias random walk  σ_b²
%
%  Each diagonal element = σ² (variance per step — already scaled by dt
%  inside ekf_propagate via the Q = B*Qimu*B'*dt formula).
%  Alternatively, a simple diagonal Q is provided here for direct use.
%
%  MEASUREMENT NOISE R:
%  ────────────────────
%  Separate R matrices for each sensor (not combined — each update is
%  applied independently):
%    ep.R_baro  (1×1)  barometer
%    ep.R_pitot (1×1)  pitot tube
%    ep.R_mag   (1×1)  magnetometer
%    ep.R_gps   (4×4)  GPS [pn; pe; Vg; chi]
%
%  INITIAL COVARIANCE P0 (13×13 diagonal):
%  ─────────────────────────────────────
%  Represents uncertainty at initialization.
%
%  TUNING NOTES:
%  ─────────────
%  • Increase Q to allow faster adaptation to model errors (more trust in
%    sensors).
%  • Decrease Q to smooth out sensor noise (more trust in model).
%  • R should match sensor sigma values from sensor_params.m.
%  • P0 sets how quickly the filter converges from a bad initial guess.
%
%  Inputs:
%    sparams  - sensor_params() struct
%    cfg      - (optional) config struct — if R_baro / R_pitot / R_mag /
%               R_gps fields are present they override the sparams defaults.
%               Use this in Real_Flight_Simulation to match actual hardware.
%               Synthetic sim omits cfg → sparams values used (consistent
%               with the sensor noise model).
%
%  Output:
%    ep       - struct with Q, R_baro, R_pitot, R_mag, R_gps, P0
%
%  Author : AtlasFC  |  Ref: Beard & McLain Ch. 8
% =========================================================================

function ep = ekf_params(sparams, cfg)

    sg = sparams.sigma_gyro;       % gyro white noise [rad/s]
    sa = sparams.sigma_accel;      % accel white noise [m/s^2]
    sb = sparams.sigma_b_gyro;     % gyro bias drive [rad/s]

    % -----------------------------------------------------------------------
    %  Process noise Q (13×13 diagonal)
    %  Continuous spectral densities, to be multiplied by dt in propagation.
    % -----------------------------------------------------------------------
    q_pos  = (sa * 0.01)^2;        % position: tiny (integrated velocity noise)
    q_vel  = sa^2;                  % velocity: accel noise PSD
    q_att  = (sg / 2)^2;           % attitude: gyro noise through kinematics
    q_bias = sb^2;                  % bias: Gauss-Markov drive variance

    ep.Q = diag([ q_pos*ones(1,3), ...    % pn, pe, pd
                  q_vel*ones(1,3), ...    % u, v, w
                  q_att*ones(1,4), ...    % e0, e1, e2, e3
                  q_bias*ones(1,3) ]);    % bp, bq, br

    % -----------------------------------------------------------------------
    %  Measurement noise R (from sensor specs)
    %  Default: derived from sensor_params (consistent with synthetic sim).
    %  Override: if cfg.R_* fields exist, use those instead (real hardware).
    % -----------------------------------------------------------------------
    ep.R_baro  = sparams.sigma_baro^2;
    ep.R_pitot = sparams.sigma_pitot^2;
    ep.R_mag   = sparams.sigma_mag^2;
    ep.R_gps   = diag([ sparams.sigma_gps_pn^2,  sparams.sigma_gps_pe^2, ...
                         sparams.sigma_gps_Vg^2,  sparams.sigma_gps_chi^2 ]);

    % Apply config overrides (Real_Flight_Simulation)
    if nargin >= 2 && ~isempty(cfg)
        if isfield(cfg, 'R_baro'),  ep.R_baro  = cfg.R_baro;  end
        if isfield(cfg, 'R_pitot'), ep.R_pitot = cfg.R_pitot; end
        if isfield(cfg, 'R_mag'),   ep.R_mag   = cfg.R_mag;   end
        if isfield(cfg, 'R_gps'),   ep.R_gps   = cfg.R_gps;   end
        if isfield(cfg, 'Q'),       ep.Q       = cfg.Q;       end
    end

    % -----------------------------------------------------------------------
    %  Initial covariance P0 (13×13 diagonal)
    % -----------------------------------------------------------------------
    ep.P0 = diag([ 10.0*ones(1,3),   ...  % position ±3 m
                    1.0*ones(1,3),   ...  % velocity ±1 m/s
                    0.01*ones(1,4),  ...  % quaternion ±0.1 rad
                    0.001*ones(1,3)  ]);  % gyro bias ±0.032 rad/s

end
