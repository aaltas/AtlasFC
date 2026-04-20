% =========================================================================
%  SENSORS_INIT - Initialize sensor state struct for AtlasFC
% =========================================================================
%  Creates a sensor_state struct with all biases, timers, and held values
%  set to their initial conditions.
%
%  Each slow sensor (baro, pitot, mag, GPS) has:
%    .xxx_timer  — countdown to next update [s]; 0 → fires on first call
%    .last_xxx   — last measurement (held between updates)
%    .xxx_new    — flag (set in sensors.m on each new reading)
%
%  Inputs:
%    sparams   - sensor_params() struct
%    x13_init  - (optional) [13x1] initial state for seeding GPS/baro/pitot
%
%  Outputs:
%    sensor_state - initialized struct
%
%  Author : AtlasFC  |  Ref: Beard & McLain Ch. 7
% =========================================================================

function sensor_state = sensors_init(sparams, x13_init)

    % --- IMU biases (Gauss-Markov, start at zero) ---
    sensor_state.gyro_bias  = zeros(3, 1);
    sensor_state.accel_bias = zeros(3, 1);

    % --- Air data / heading biases ---
    sensor_state.baro_bias  = 0;
    sensor_state.pitot_bias = 0;
    % (magnetometer has no Gauss-Markov bias in simplified Ch.7 model)

    % --- GPS bias (4x1: pn, pe, Vg, chi) ---
    sensor_state.gps_bias   = zeros(4, 1);

    % --- Update timers (0 → fires immediately on first call) ---
    sensor_state.baro_timer  = 0;
    sensor_state.pitot_timer = 0;
    sensor_state.mag_timer   = 0;
    sensor_state.gps_timer   = 0;

    % --- Held (last) measurements ---
    sensor_state.last_baro  = 0;
    sensor_state.last_pitot = 0;
    sensor_state.last_mag   = 0;
    sensor_state.last_gps   = zeros(4, 1);

    % Seed from initial state if provided
    if nargin >= 2 && ~isempty(x13_init)
        Va0 = sqrt(x13_init(4)^2 + x13_init(5)^2 + x13_init(6)^2);
        R_bv0 = quaternion_to_rotation(x13_init(7:10));
        [~, ~, psi0] = rotation_to_euler(R_bv0);

        sensor_state.last_baro  = -x13_init(3);       % h = -pd
        sensor_state.last_pitot = Va0;
        sensor_state.last_mag   = psi0;
        sensor_state.last_gps   = [x13_init(1); x13_init(2); Va0; psi0];
    end

    % --- New-measurement flags (false until first update fires) ---
    sensor_state.baro_new  = false;
    sensor_state.pitot_new = false;
    sensor_state.mag_new   = false;
    sensor_state.gps_new   = false;

end
