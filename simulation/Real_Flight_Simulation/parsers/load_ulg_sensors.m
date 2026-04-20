% =========================================================================
%  LOAD_ULG_SENSORS  —  Reading raw sensor data from PX4 ulog
% =========================================================================
%  Extracts IMU, baro, GPS and magnetometer data from .ulg file with
%  ulogreader, converts units and resamples to cfg.dt time step.
%
%  PX4 topic → our EKF format:
%    sensor_gyro      → y_gyro  [rad/s]        (x,y,z)
%    sensor_accel     → y_accel [m/s²]          (x,y,z)
%    sensor_baro      → y_baro  [m]             (altitude from pressure)
%    vehicle_gps_position → y_gps  [m, m/s, rad] (pN,pE, Va, psi_gps)
%    sensor_mag       → y_mag   [rad]           (yaw approximation with atan2)
%
%  GPS coordinates are converted to NED local position relative to first fix point.
%
%  Inputs:
%    ulog   ulogreader object
%    cfg    flight_cfg() struct  (.dt, .t_start, .t_end, .gps_instance, etc.)
%
%  Outputs:
%    sensors  struct
%      .t          [1×N]  common time vector [s]
%      .gyro       [3×N]  [rad/s]
%      .accel      [3×N]  [m/s²]
%      .baro       [1×N]  altitude [m]
%      .baro_valid [1×N]  logical
%      .gps        [4×N]  [pN(m), pE(m), Va(m/s), psi(rad)]
%      .gps_valid  [1×N]  logical
%      .mag        [1×N]  yaw estimate [rad]
%      .mag_valid  [1×N]  logical
%      .lat0       reference latitude  [rad]
%      .lon0       reference longitude [rad]
%      .alt0       reference altitude [m]
%
%  Author : AtlasFC
% =========================================================================

function sensors = load_ulg_sensors(ulog, cfg)

    fprintf('  [sensors] Reading raw sensor data...\n');

    % -----------------------------------------------------------------------
    %  1. Read each topic
    % -----------------------------------------------------------------------

    % --- Gyro ---
    gyro_raw = readTopicMsgs(ulog, 'TopicNames', {'sensor_gyro'}, ...
                             'InstanceID', {cfg.imu_instance});
    gyro_tbl = gyro_raw.TopicMessages{1};
    t_gyro   = seconds(gyro_tbl.timestamp);          % duration → s
    gx = double(gyro_tbl.x);
    gy = double(gyro_tbl.y);
    gz = double(gyro_tbl.z);

    % --- Accel ---
    acc_raw  = readTopicMsgs(ulog, 'TopicNames', {'sensor_accel'}, ...
                             'InstanceID', {cfg.imu_instance});
    acc_tbl  = acc_raw.TopicMessages{1};
    t_accel  = seconds(acc_tbl.timestamp);
    ax = double(acc_tbl.x);
    ay = double(acc_tbl.y);
    az = double(acc_tbl.z);

    % --- Baro ---
    baro_raw = readTopicMsgs(ulog, 'TopicNames', {'sensor_baro'}, ...
                             'InstanceID', {0});
    baro_tbl = baro_raw.TopicMessages{1};
    t_baro   = seconds(baro_tbl.timestamp);
    % Altitude from pressure: ISA sea level model
    P_baro   = double(baro_tbl.pressure);         % [Pa]
    P0       = 101325.0;                           % [Pa] sea level
    h_baro   = 44330.0 * (1 - (P_baro / P0).^(1/5.255));  % [m]

    % --- GPS ---
    gps_raw  = readTopicMsgs(ulog, 'TopicNames', {'vehicle_gps_position'}, ...
                             'InstanceID', {0});
    gps_tbl  = gps_raw.TopicMessages{1};
    t_gps    = seconds(gps_tbl.timestamp);
    lat_deg  = double(gps_tbl.lat) * 1e-7;        % [deg]
    lon_deg  = double(gps_tbl.lon) * 1e-7;        % [deg]
    alt_m    = double(gps_tbl.alt) * 1e-3;        % mm → m
    vel_n    = double(gps_tbl.vel_n_m_s);         % [m/s]
    vel_e    = double(gps_tbl.vel_e_m_s);         % [m/s]

    % GPS reference point (first valid fix)
    fix_ok   = double(gps_tbl.fix_type) >= 3;
    i_ref    = find(fix_ok, 1, 'first');
    if isempty(i_ref)
        error('No GPS fix found — check flight data.');
    end
    lat0 = deg2rad(lat_deg(i_ref));
    lon0 = deg2rad(lon_deg(i_ref));
    alt0 = alt_m(i_ref);
    Re   = 6371000.0;   % [m] mean Earth radius

    lat_rad = deg2rad(lat_deg);
    lon_rad = deg2rad(lon_deg);
    pN_gps  =  Re * (lat_rad - lat0);
    pE_gps  =  Re * cos(lat0) * (lon_rad - lon0);

    % Airspeed approximation from GPS velocity (assuming no wind)
    Va_gps   = sqrt(vel_n.^2 + vel_e.^2);
    psi_gps  = atan2(vel_e, vel_n);               % course angle [rad]

    % --- Magnetometer ---
    mag_raw  = readTopicMsgs(ulog, 'TopicNames', {'sensor_mag'}, ...
                             'InstanceID', {0});
    mag_tbl  = mag_raw.TopicMessages{1};
    t_mag    = seconds(mag_tbl.timestamp);
    mx       = double(mag_tbl.x);
    my       = double(mag_tbl.y);
    psi_mag  = atan2(my, mx);                     % yaw approximation [rad]

    % -----------------------------------------------------------------------
    %  2. Time window — cfg.t_start / t_end absolute log time
    % -----------------------------------------------------------------------
    t0_log = t_gyro(1);    % log start reference

    if isnan(cfg.t_start)
        t_begin = 0;
    else
        t_begin = cfg.t_start;
    end
    if isnan(cfg.t_end)
        t_end_s = t_gyro(end) - t0_log;
    else
        t_end_s = cfg.t_end;
    end

    % -----------------------------------------------------------------------
    %  3. Auto-detect actual sensor periods from log timestamps
    %
    %  Rather than trusting cfg.dt or cfg.gps_dt as update periods,
    %  we compute the median inter-sample interval directly from the
    %  raw timestamps.  This ensures valid flags match the real
    %  hardware rate regardless of what is set in flight_cfg.m.
    % -----------------------------------------------------------------------
    dt_baro_actual = median(diff(t_baro));    % [s]  actual baro period
    dt_gps_actual  = median(diff(t_gps));     % [s]  actual GPS period
    dt_mag_actual  = median(diff(t_mag));     % [s]  actual mag period
    dt_imu_actual  = median(diff(t_gyro));    % [s]  actual IMU period

    fprintf('  [sensors] Detected rates  — IMU: %.0f Hz  Baro: %.0f Hz  GPS: %.1f Hz  Mag: %.0f Hz\n', ...
            1/dt_imu_actual, 1/dt_baro_actual, 1/dt_gps_actual, 1/dt_mag_actual);

    % -----------------------------------------------------------------------
    %  4. Common time grid (EKF step = cfg.dt, independent of IMU rate)
    %
    %  Note: even if IMU is at 400 Hz, the EKF propagates at cfg.dt.
    %  Q_d = ep.Q * cfg.dt in ekf_propagate.m — already scales correctly.
    %  The IMU data is averaged over each cfg.dt window (linear interp),
    %  which is equivalent to a box filter at the cfg.dt rate.
    % -----------------------------------------------------------------------
    t_grid = (t_begin : cfg.dt : t_end_s)';
    N      = length(t_grid);

    % Normalize all timestamps to t0
    t_gyro  = t_gyro  - t0_log;
    t_accel = t_accel - t0_log;
    t_baro  = t_baro  - t0_log;
    t_gps   = t_gps   - t0_log;
    t_mag   = t_mag   - t0_log;

    % -----------------------------------------------------------------------
    %  5. Interpolation / nearest neighbor resampling
    %     valid flags use auto-detected periods — not cfg values
    % -----------------------------------------------------------------------

    % IMU: linear interpolation onto EKF grid (handles any source rate)
    gyro_out  = [interp1(t_gyro, gx, t_grid, 'linear', 'extrap'), ...
                 interp1(t_gyro, gy, t_grid, 'linear', 'extrap'), ...
                 interp1(t_gyro, gz, t_grid, 'linear', 'extrap')]';   % 3×N

    accel_out = [interp1(t_accel, ax, t_grid, 'linear', 'extrap'), ...
                 interp1(t_accel, ay, t_grid, 'linear', 'extrap'), ...
                 interp1(t_accel, az, t_grid, 'linear', 'extrap')]';  % 3×N

    % Baro: use actual period from log
    [baro_out, baro_valid] = nearest_resample(t_baro, h_baro,   t_grid, dt_baro_actual);

    % GPS: use actual period from log
    [gps_pN_out, gps_valid_n] = nearest_resample(t_gps, pN_gps,  t_grid, dt_gps_actual);
    [gps_pE_out, ~]           = nearest_resample(t_gps, pE_gps,  t_grid, dt_gps_actual);
    [gps_Va_out, ~]           = nearest_resample(t_gps, Va_gps,  t_grid, dt_gps_actual);
    [gps_ps_out, ~]           = nearest_resample(t_gps, psi_gps, t_grid, dt_gps_actual);
    gps_valid                 = gps_valid_n;

    % Mag: use actual period from log
    [mag_out, mag_valid] = nearest_resample(t_mag, psi_mag, t_grid, dt_mag_actual);

    % -----------------------------------------------------------------------
    %  6. Pack  (include detected rates for downstream use)
    % -----------------------------------------------------------------------
    sensors.t         = t_grid';          % 1×N
    sensors.gyro      = gyro_out;         % 3×N
    sensors.accel     = accel_out;        % 3×N
    sensors.baro      = baro_out;         % 1×N
    sensors.baro_valid= baro_valid;       % 1×N logical
    sensors.gps       = [gps_pN_out; gps_pE_out; gps_Va_out; gps_ps_out]; % 4×N
    sensors.gps_valid = gps_valid;        % 1×N logical
    sensors.mag       = mag_out;          % 1×N
    sensors.mag_valid = mag_valid;        % 1×N logical
    sensors.lat0      = lat0;
    sensors.lon0      = lon0;
    sensors.alt0      = alt0;

    % Detected rates — used by run_flight_eval for stats and ekf_params update
    sensors.rate.imu  = 1 / dt_imu_actual;
    sensors.rate.baro = 1 / dt_baro_actual;
    sensors.rate.gps  = 1 / dt_gps_actual;
    sensors.rate.mag  = 1 / dt_mag_actual;

    fprintf('  [sensors] Done — %d steps, %.1f s\n', N, t_grid(end));
end

% =========================================================================
%  LOCAL HELPERS
% =========================================================================

function [out, valid] = nearest_resample(t_src, v_src, t_grid, update_dt)
% Selects nearest sample; marks valid=true only when new sample
% arrives within update_dt delay (simulates discrete sensor update).
    N     = length(t_grid);
    out   = zeros(1, N);
    valid = false(1, N);
    t_last_update = -inf;

    for k = 1:N
        t_now = t_grid(k);
        % Find nearest source sample
        [~, idx] = min(abs(t_src - t_now));
        out(k)   = v_src(idx);
        % Did new update arrive?
        if (t_now - t_last_update) >= update_dt * 0.99
            valid(k)      = true;
            t_last_update = t_now;
        end
    end
end
