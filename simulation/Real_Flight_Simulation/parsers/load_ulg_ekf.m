% =========================================================================
%  LOAD_ULG_EKF  —  Reading PX4 EKF2 output as reference
% =========================================================================
%  Extracts PX4's own EKF2 estimates from ulog and aligns to sensors.t
%  time grid.  This data is used as "reference" for comparing our own EKF.
%
%  PX4 topics read:
%    vehicle_local_position  → pN, pE, pD, vN, vE, vD  (NED, [m], [m/s])
%    vehicle_attitude        → quaternion → phi, theta, psi
%    estimator_status        → innovation, variance info (optional)
%
%  Inputs:
%    ulog      ulogreader object
%    t_grid    [1×N] common time vector with sensors [s]  (sensors.t)
%    t0_log    log start time [s] (from load_ulg_sensors)
%
%  Outputs:
%    px4  struct
%      .t       [1×N]   time [s]
%      .pN      [1×N]   north position [m]
%      .pE      [1×N]   east position [m]
%      .pD      [1×N]   down position [m]  (h = -pD)
%      .vN      [1×N]   north velocity [m/s]
%      .vE      [1×N]   east velocity [m/s]
%      .vD      [1×N]   down velocity [m/s]
%      .phi     [1×N]   roll [rad]
%      .theta   [1×N]   pitch [rad]
%      .psi     [1×N]   yaw [rad]
%      .Va      [1×N]   airspeed approximation [m/s]  (sqrt(vN²+vE²))
%      .h       [1×N]   altitude [m]  (-pD)
%
%  Author : AtlasFC
% =========================================================================

function px4 = load_ulg_ekf(ulog, t_grid, t0_log)

    fprintf('  [px4_ekf] Reading PX4 EKF2 reference data...\n');

    % -----------------------------------------------------------------------
    %  1. vehicle_local_position  (position + velocity)
    % -----------------------------------------------------------------------
    lpos_raw = readTopicMsgs(ulog, 'TopicNames', {'vehicle_local_position'}, ...
                             'InstanceID', {0});
    lpos     = lpos_raw.TopicMessages{1};
    t_lpos   = seconds(lpos.timestamp) - t0_log;

    pN_px4 = double(lpos.x);
    pE_px4 = double(lpos.y);
    pD_px4 = double(lpos.z);
    vN_px4 = double(lpos.vx);
    vE_px4 = double(lpos.vy);
    vD_px4 = double(lpos.vz);

    % -----------------------------------------------------------------------
    %  2. vehicle_attitude  (attitude quaternion)
    % -----------------------------------------------------------------------
    att_raw = readTopicMsgs(ulog, 'TopicNames', {'vehicle_attitude'}, ...
                            'InstanceID', {0});
    att     = att_raw.TopicMessages{1};
    t_att   = seconds(att.timestamp) - t0_log;

    % PX4 quaternion: q[0]=w, q[1]=x, q[2]=y, q[3]=z  (Hamilton convention)
    q_w = double(att.q(:,1));
    q_x = double(att.q(:,2));
    q_y = double(att.q(:,3));
    q_z = double(att.q(:,4));

    % Quaternion → Euler (ZYX / NED-body convention)
    phi_px4   = atan2(2*(q_w.*q_x + q_y.*q_z), 1 - 2*(q_x.^2 + q_y.^2));
    theta_px4 = asin( max(-1, min(1, 2*(q_w.*q_y - q_z.*q_x))) );
    psi_px4   = atan2(2*(q_w.*q_z + q_x.*q_y), 1 - 2*(q_y.^2 + q_z.^2));

    % -----------------------------------------------------------------------
    %  3. Interpolate to common time grid
    % -----------------------------------------------------------------------
    t_q = t_grid;   % column vector

    px4.t     = t_grid;
    px4.pN    = interp1(t_lpos, pN_px4, t_q, 'linear', 'extrap')';
    px4.pE    = interp1(t_lpos, pE_px4, t_q, 'linear', 'extrap')';
    px4.pD    = interp1(t_lpos, pD_px4, t_q, 'linear', 'extrap')';
    px4.vN    = interp1(t_lpos, vN_px4, t_q, 'linear', 'extrap')';
    px4.vE    = interp1(t_lpos, vE_px4, t_q, 'linear', 'extrap')';
    px4.vD    = interp1(t_lpos, vD_px4, t_q, 'linear', 'extrap')';
    px4.phi   = interp1(t_att,  phi_px4,   t_q, 'linear', 'extrap')';
    px4.theta = interp1(t_att,  theta_px4, t_q, 'linear', 'extrap')';
    px4.psi   = interp1(t_att,  psi_px4,   t_q, 'linear', 'extrap')';

    % Derived quantities
    px4.Va = sqrt(px4.vN.^2 + px4.vE.^2);   % horizontal velocity ≈ airspeed
    px4.h  = -px4.pD;                         % altitude [m]

    fprintf('  [px4_ekf] Done — %d steps interpolation completed.\n', length(t_grid));
end
