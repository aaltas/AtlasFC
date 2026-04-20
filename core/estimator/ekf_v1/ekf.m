% =========================================================================
%  EKF - Top-level 13-state INS/GPS Extended Kalman Filter
% =========================================================================
%  Orchestrates prediction and sensor-triggered measurement updates.
%  Called ONCE per IMU step (100 Hz).  Slower sensors (baro 25 Hz,
%  pitot 25 Hz, mag 10 Hz, GPS 1 Hz) are updated only when their
%  _new flag is true in the sensor measurement struct y.
%
%  ADDING A NEW SENSOR:
%    1. Write meas_xxx.m  →  returns [h_pred, H]
%    2. Add R_xxx field to ekf_params.m
%    3. Add 3 lines here (if y.xxx_new → ekf_update)
%
%  ARCHITECTURE:
%    y.gyro  + y.accel  →  ekf_propagate()    [every step]
%    y.baro  (baro_new) →  ekf_update(meas_baro)
%    y.pitot (pitot_new)→  ekf_update(meas_pitot)
%    y.mag   (mag_new)  →  ekf_update(meas_mag)   [wrap chi]
%    y.gps   (gps_new)  →  ekf_update(meas_gps)   [wrap chi]
%
%  Inputs:
%    x_hat    [13x1]  current state estimate
%    P        [13x13] current covariance
%    y        struct  sensor measurements from sensors.m (with _new flags)
%    ep       struct  from ekf_params()
%    params   struct  mav_params()
%    dt       [s]     IMU time step
%
%  Outputs:
%    x_hat    [13x1]  updated state estimate
%    P        [13x13] updated covariance
%    innov    struct  innovation vectors (.baro .pitot .mag .gps) for logging
%
%  Usage:
%    ep           = ekf_params(sparams);
%    [x_hat, P]   = ekf_init(x13_trim, ep);
%    for k = 1:N
%        [y, sensor_state] = sensors(x13, fm, sensor_state, sparams, params, dt);
%        [x_hat, P, innov] = ekf(x_hat, P, y, ep, params, dt);
%        ... use x_hat as estimated state
%    end
%
%  Author : AtlasFC  |  Ref: Beard & McLain Ch. 8
% =========================================================================

function [x_hat, P, innov] = ekf(x_hat, P, y, ep, params, dt)

    innov.baro  = 0;
    innov.pitot = 0;
    innov.mag   = 0;
    innov.gps   = zeros(4, 1);

    % ====================================================================
    %  1. PROPAGATION — every IMU step (100 Hz)
    % ====================================================================
    [x_hat, P] = ekf_propagate(x_hat, P, y.gyro, y.accel, ep, params, dt);

    % ====================================================================
    %  2. BAROMETER update — 25 Hz
    % ====================================================================
    if y.baro_new
        [h_pred, H] = meas_baro(x_hat);
        [x_hat, P, innov.baro] = ekf_update(x_hat, P, H, ep.R_baro, ...
                                              y.baro, h_pred);
    end

    % ====================================================================
    %  3. PITOT update — 25 Hz
    % ====================================================================
    if y.pitot_new
        [h_pred, H] = meas_pitot(x_hat);
        [x_hat, P, innov.pitot] = ekf_update(x_hat, P, H, ep.R_pitot, ...
                                               y.pitot, h_pred);
    end

    % ====================================================================
    %  4. MAGNETOMETER update — 10 Hz  (angle-wrapped innovation)
    % ====================================================================
    if y.mag_new
        [h_pred, H] = meas_mag(x_hat);
        [x_hat, P, innov.mag] = ekf_update(x_hat, P, H, ep.R_mag, ...
                                             y.mag, h_pred, 1);   % wrap_idx=1
    end

    % ====================================================================
    %  5. GPS update — 1 Hz  (angle-wrapped chi innovation)
    % ====================================================================
    if y.gps_new
        [h_pred, H] = meas_gps(x_hat);
        [x_hat, P, innov.gps] = ekf_update(x_hat, P, H, ep.R_gps, ...
                                             y.gps, h_pred, 4);   % wrap_idx=4 (chi)
    end

end
