% =========================================================================
%  SENSOR_PARAMS - Sensor noise and bias parameters for AtlasFC
% =========================================================================
%  Defines all sensor error characteristics used in Chapter 7 sensor
%  models and Chapter 8 state estimation.
%
%  Each sensor has:
%    sigma_*  - standard deviation of white (Gaussian) measurement noise
%    tau_*    - Gauss-Markov bias time constant [s]
%    sigma_b* - standard deviation of bias noise drive (Gauss-Markov)
%
%  Gauss-Markov bias model (discrete):
%    b_k+1 = exp(-dt/tau)*b_k + sigma_b*sqrt(1-exp(-2*dt/tau))*randn
%
%  GPS update rate is separate (typically 1 Hz).
%
%  Usage:
%    sparams = sensor_params();
%
%  To tune for a new aircraft, scale sigma values by sensor data sheet
%  noise density, and tau by the observed bias drift time scale.
%
%  Reference: Beard & McLain, "Small Unmanned Aircraft", Ch. 7
%  Author   : AtlasFC
% =========================================================================

function sparams = sensor_params()

    % -----------------------------------------------------------------------
    %  RATE GYROSCOPE (measured: p, q, r in body frame)
    % -----------------------------------------------------------------------
    sparams.sigma_gyro  = deg2rad(0.13);   % rad/s  — white noise std dev (~MEMS grade)
    sparams.tau_gyro    = 300.0;           % s      — bias time constant (slow drift)
    sparams.sigma_b_gyro = deg2rad(0.004); % rad/s  — Gauss-Markov bias noise drive

    % -----------------------------------------------------------------------
    %  ACCELEROMETER (measured: specific force fx, fy, fz in body frame)
    % -----------------------------------------------------------------------
    sparams.sigma_accel  = 0.0025 * 9.81;   % m/s^2 — white noise std dev (~0.25 mg)
    sparams.tau_accel    = 300.0;            % s     — bias time constant
    sparams.sigma_b_accel = 0.00025 * 9.81; % m/s^2 — Gauss-Markov bias noise drive

    % -----------------------------------------------------------------------
    %  BAROMETRIC ALTIMETER (measured: altitude h = -pd)
    % -----------------------------------------------------------------------
    sparams.sigma_baro  = 0.10;            % m  — white noise std dev
    sparams.tau_baro    = 200.0;           % s  — bias time constant
    sparams.sigma_b_baro = 0.01;          % m  — Gauss-Markov bias noise drive

    % -----------------------------------------------------------------------
    %  PITOT TUBE (measured: airspeed Va)
    % -----------------------------------------------------------------------
    sparams.sigma_pitot  = 0.01;           % m/s — white noise std dev
    sparams.tau_pitot    = 200.0;          % s   — bias time constant
    sparams.sigma_b_pitot = 0.001;         % m/s — Gauss-Markov bias noise drive

    % -----------------------------------------------------------------------
    %  MAGNETOMETER (measured: heading angle psi)
    % -----------------------------------------------------------------------
    sparams.sigma_mag    = deg2rad(1.0);   % rad — white noise std dev (~1 deg)
    % Note: magnetometer bias is assumed constant (not Gauss-Markov)
    % in the simplified B&M Ch.7 model.  No tau/sigma_b defined here.

    % -----------------------------------------------------------------------
    %  GPS (measured: pn, pe, Vg, chi — ground position and velocity)
    % -----------------------------------------------------------------------
    sparams.sigma_gps_pn  = 0.21;          % m    — north position noise std dev
    sparams.sigma_gps_pe  = 0.21;          % m    — east  position noise std dev
    sparams.sigma_gps_Vg  = 0.05;          % m/s  — ground speed noise std dev
    sparams.sigma_gps_chi = deg2rad(0.5);  % rad  — course angle noise std dev

    sparams.tau_gps_pn   = 1e8;            % s    — very slow bias drift (static bias ok)
    sparams.tau_gps_pe   = 1e8;            % s
    sparams.tau_gps_Vg   = 1e8;            % s
    sparams.tau_gps_chi  = 1e8;            % s

    sparams.sigma_b_gps_pn  = 0.21;       % m   — bias noise drive
    sparams.sigma_b_gps_pe  = 0.21;       % m
    sparams.sigma_b_gps_Vg  = 0.05;       % m/s
    sparams.sigma_b_gps_chi = deg2rad(0.5);% rad

    % GPS update rate
    sparams.gps_hz = 1.0;                  % Hz — how often GPS gives a new fix

    % -----------------------------------------------------------------------
    %  AIR DATA & HEADING UPDATE RATES
    % -----------------------------------------------------------------------
    sparams.baro_hz  = 25.0;   % Hz — barometric altimeter
    sparams.pitot_hz = 25.0;   % Hz — pitot tube airspeed
    sparams.mag_hz   = 10.0;   % Hz — magnetometer heading

end
