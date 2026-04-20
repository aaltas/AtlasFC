% =========================================================================
%  SENSORS - Top-level sensor suite (rate-limited, flag-based)
% =========================================================================
%  Called at IMU rate (100 Hz).  Each slower sensor (baro, pitot, mag,
%  GPS) has its own countdown timer.  When the timer expires a new
%  measurement is generated; between firings the last measurement is held
%  (zero-order hold) and the corresponding _new flag is false.
%
%  RATES (from sensor_params):
%    IMU  (gyro + accel) : 100 Hz  — every call
%    Baro, Pitot         :  25 Hz  — timer 0.04 s
%    Magnetometer        :  10 Hz  — timer 0.10 s
%    GPS                 :   1 Hz  — timer 1.00 s
%
%  OUTPUT STRUCT y:
%    .gyro   [3x1]  angular rates [rad/s]
%    .accel  [3x1]  specific force [m/s^2]
%    .baro   [1x1]  altitude [m]          + .baro_new  flag
%    .pitot  [1x1]  airspeed [m/s]        + .pitot_new flag
%    .mag    [1x1]  heading [rad]         + .mag_new   flag
%    .gps    [4x1]  [pn;pe;Vg;chi]        + .gps_new   flag
%
%  Inputs:
%    x13          - 13-state vector
%    fm           - [6x1] forces+moments
%    sensor_state - state struct from sensors_init()
%    sparams      - sensor_params() struct
%    params       - mav_params() struct
%    dt           - simulation time step [s]
%
%  Outputs:
%    y            - measurement struct (with _new flags)
%    sensor_state - updated state struct
%
%  Author : AtlasFC  |  Ref: Beard & McLain Ch. 7
% =========================================================================

function [y, sensor_state] = sensors(x13, fm, sensor_state, sparams, params, dt)

    Va_true = sqrt(x13(4)^2 + x13(5)^2 + x13(6)^2);

    % ====================================================================
    %  1. IMU — runs every call (100 Hz)
    % ====================================================================
    [y.gyro, sensor_state.gyro_bias] = ...
        gyro_model(x13, sensor_state.gyro_bias, sparams, dt);

    [y.accel, sensor_state.accel_bias] = ...
        accel_model(x13, fm, sensor_state.accel_bias, sparams, params, dt);

    % ====================================================================
    %  2. BAROMETER — 25 Hz
    % ====================================================================
    sensor_state.baro_timer = sensor_state.baro_timer - dt;
    if sensor_state.baro_timer <= 0
        [sensor_state.last_baro, sensor_state.baro_bias] = ...
            baro_model(x13, sensor_state.baro_bias, sparams, dt);
        sensor_state.baro_timer = 1.0 / sparams.baro_hz;
        sensor_state.baro_new   = true;
    else
        sensor_state.baro_new = false;
    end
    y.baro     = sensor_state.last_baro;
    y.baro_new = sensor_state.baro_new;

    % ====================================================================
    %  3. PITOT TUBE — 25 Hz
    % ====================================================================
    sensor_state.pitot_timer = sensor_state.pitot_timer - dt;
    if sensor_state.pitot_timer <= 0
        [sensor_state.last_pitot, sensor_state.pitot_bias] = ...
            pitot_model(x13, Va_true, sensor_state.pitot_bias, sparams, dt);
        sensor_state.pitot_timer = 1.0 / sparams.pitot_hz;
        sensor_state.pitot_new   = true;
    else
        sensor_state.pitot_new = false;
    end
    y.pitot     = sensor_state.last_pitot;
    y.pitot_new = sensor_state.pitot_new;

    % ====================================================================
    %  4. MAGNETOMETER — 10 Hz
    % ====================================================================
    sensor_state.mag_timer = sensor_state.mag_timer - dt;
    if sensor_state.mag_timer <= 0
        sensor_state.last_mag  = mag_model(x13, sparams);
        sensor_state.mag_timer = 1.0 / sparams.mag_hz;
        sensor_state.mag_new   = true;
    else
        sensor_state.mag_new = false;
    end
    y.mag     = sensor_state.last_mag;
    y.mag_new = sensor_state.mag_new;

    % ====================================================================
    %  5. GPS — 1 Hz
    % ====================================================================
    sensor_state.gps_timer = sensor_state.gps_timer - dt;
    if sensor_state.gps_timer <= 0
        % Gauss-Markov bias propagation
        tau_v   = [sparams.tau_gps_pn; sparams.tau_gps_pe;
                   sparams.tau_gps_Vg; sparams.tau_gps_chi];
        sig_b_v = [sparams.sigma_b_gps_pn; sparams.sigma_b_gps_pe;
                   sparams.sigma_b_gps_Vg; sparams.sigma_b_gps_chi];
        alpha_v = exp(-dt ./ tau_v);
        sig_d   = sig_b_v .* sqrt(1 - alpha_v.^2);
        sensor_state.gps_bias = alpha_v .* sensor_state.gps_bias + sig_d .* randn(4,1);

        % True states
        R_bv  = quaternion_to_rotation(x13(7:10));
        [~, ~, psi_true] = rotation_to_euler(R_bv);
        sig_v = [sparams.sigma_gps_pn; sparams.sigma_gps_pe;
                 sparams.sigma_gps_Vg; sparams.sigma_gps_chi];
        true_v = [x13(1); x13(2); Va_true; psi_true];
        meas   = true_v + sensor_state.gps_bias + sig_v .* randn(4,1);
        meas(4) = atan2(sin(meas(4)), cos(meas(4)));   % wrap chi

        sensor_state.last_gps  = meas;
        sensor_state.gps_timer = 1.0 / sparams.gps_hz;
        sensor_state.gps_new   = true;
    else
        sensor_state.gps_new = false;
    end
    y.gps     = sensor_state.last_gps;
    y.gps_new = sensor_state.gps_new;

end
