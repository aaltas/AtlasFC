% =========================================================================
%  FLIGHT_CFG  —  Real_Flight_Simulation configuration
% =========================================================================
%  Called by run_flight_eval.m.
%
%  Usage:
%    cfg = flight_cfg();
%
%  Author : AtlasFC
% =========================================================================

function cfg = flight_cfg()

    % -----------------------------------------------------------------------
    %  Log file
    % -----------------------------------------------------------------------
    cfg.ulg_file = '13_24_30.ulg';   % searched in log_files/ folder
                                      % enter filename only, path is
                                      % added automatically by run_flight_eval.m

    % -----------------------------------------------------------------------
    %  Time window  [s]  (relative to log start)
    %    NaN → entire log is used
    % -----------------------------------------------------------------------
    cfg.t_start = NaN;   % [s]  NaN = from log start
    cfg.t_end   = NaN;   % [s]  NaN = to log end

    % -----------------------------------------------------------------------
    %  Resampling
    % -----------------------------------------------------------------------
    cfg.dt      = 0.01;   % [s]  common time step  (100 Hz, same as EKF)
    cfg.gps_dt  = 0.2;    % [s]  GPS update period (5 Hz)

    % -----------------------------------------------------------------------
    %  IMU instance  (change if multiple IMUs)
    % -----------------------------------------------------------------------
    cfg.imu_instance = 0;   % sensor_gyro / sensor_accel instance ID

    % -----------------------------------------------------------------------
    %  EKF initial perturbation  (0 → perfect init)
    % -----------------------------------------------------------------------
    cfg.ekf_pos_err = 0.0;   % [m]
    cfg.ekf_vel_err = 0.0;   % [m/s]

    % -----------------------------------------------------------------------
    %  EKF version  — same selector as Synthetic_Flight_Simulation.
    %  Change to 'ekf_v2' to compare new estimator against PX4 on real data.
    % -----------------------------------------------------------------------
    cfg.ekf_version = 'ekf_v1';   % 'ekf_v1' | 'ekf_v2' | ...

    % -----------------------------------------------------------------------
    %  EKF measurement noise R  —  real hardware overrides
    % -----------------------------------------------------------------------
    %  These values replace the sensor_params defaults inside ekf_params.m.
    %  Tune them to match the actual noise of the hardware in the log.
    %
    %  How to estimate:
    %    Baro / Pitot / Mag : take a static ground recording, compute std()
    %    GPS pos            : check datasheet or use eph/epv fields in log
    %    GPS course         : std(atan2(vel_e, vel_n)) during straight flight
    %
    %  Comment out any field to fall back to the sensor_params default.
    % -----------------------------------------------------------------------
    cfg.R_baro  = 0.5^2;                              % [m²]    baro altitude noise
    cfg.R_pitot = 1.0^2;                              % [m²/s²] airspeed noise
    cfg.R_mag   = deg2rad(5)^2;                       % [rad²]  yaw from mag noise
    cfg.R_gps   = diag([1.5^2, 1.5^2, 0.5^2, deg2rad(3)^2]);
    %              [pN(m²), pE(m²), Va(m²/s²), chi(rad²)]

    % -----------------------------------------------------------------------
    %  Comparison metrics
    % -----------------------------------------------------------------------
    cfg.rmse_burn_in = 5.0;   % [s]  exclude first N seconds from RMSE calculation
                               %       (EKF convergence period)
end
