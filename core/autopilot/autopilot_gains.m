% =========================================================================
%  AUTOPILOT_GAINS - Compute PID gains from transfer function coefficients
% =========================================================================
%  Uses the analytical gain formulas from Beard & McLain Ch.6 to compute
%  PID / PD gains for each autopilot channel from the a-coefficients
%  produced by transfer_functions().
%
%  DESIGN PHILOSOPHY (Successive Loop Closure):
%    Each channel is designed independently using the simplified single-
%    channel transfer function. Inner loops are designed first (fast), outer
%    loops second (slow). Each outer loop assumes the inner loop tracks
%    perfectly within its bandwidth.
%
%  CHANNEL HIERARCHY:
%    LATERAL:
%      Inner: Roll hold    φ_ref → δa    (PD,  ωn ≈ 10 rad/s)
%      Outer: Course hold  χ_ref → φ_ref  (P,   ωn ≈ 0.5 rad/s)
%
%    LONGITUDINAL:
%      Inner: Pitch hold   θ_ref → δe    (PD,  ωn ≈ 5 rad/s)
%      Outer: Alt hold     h_ref → θ_ref  (P,   ωn ≈ 0.2 rad/s)
%      Para:  Va hold      Va_ref → δt   (PI,  ωn ≈ 1 rad/s)
%
%  FORMULA DERIVATIONS (see Beard & McLain Ch.6):
%    Roll:     Char. poly: s² + (a_φ1 − a_φ2·kd)s + a_φ2·kp = 0
%    Pitch:    Char. poly: s² + (a_θ1 − a_θ3·kd)s + (a_θ2 − a_θ3·kp) = 0
%    Airspeed: Char. poly: s² + (a_V1 + a_V2·kp)s + a_V2·ki = 0
%
%  Inputs:
%    tf_s   - struct from transfer_functions()
%    params - struct from mav_params()
%
%  Optional: override default bandwidths by passing a tuning struct
%    tuning.wn_phi, tuning.zeta_phi
%    tuning.wn_theta, tuning.zeta_theta
%    tuning.wn_V, tuning.zeta_V
%    tuning.wn_h, tuning.wn_chi
%
%  Output:
%    gains  - struct with all PID gains
%
%  Usage:
%    gains = autopilot_gains(tf_s, params);
%    gains = autopilot_gains(tf_s, params, tuning);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 6
% =========================================================================

function gains = autopilot_gains(tf_s, params, tuning)

    % -----------------------------------------------------------------------
    % DEFAULT BANDWIDTH TARGETS
    % -----------------------------------------------------------------------
    wn_phi   = 10.0;   zeta_phi   = 0.70;   % roll attitude
    wn_theta =  5.0;   zeta_theta = 0.85;   % pitch attitude
    wn_V     =  1.0;   zeta_V     = 0.90;   % airspeed via throttle
    wn_h     =  0.20;                        % altitude via pitch
    wn_chi   =  0.50;                        % course via roll

    % Override from optional tuning struct
    if nargin >= 3 && isstruct(tuning)
        if isfield(tuning, 'wn_phi'),   wn_phi   = tuning.wn_phi;   end
        if isfield(tuning, 'zeta_phi'), zeta_phi = tuning.zeta_phi; end
        if isfield(tuning, 'wn_theta'), wn_theta = tuning.wn_theta; end
        if isfield(tuning, 'zeta_theta'), zeta_theta = tuning.zeta_theta; end
        if isfield(tuning, 'wn_V'),     wn_V     = tuning.wn_V;     end
        if isfield(tuning, 'zeta_V'),   zeta_V   = tuning.zeta_V;   end
        if isfield(tuning, 'wn_h'),     wn_h     = tuning.wn_h;     end
        if isfield(tuning, 'wn_chi'),   wn_chi   = tuning.wn_chi;   end
    end

    % -----------------------------------------------------------------------
    % ROLL CHANNEL  (Beard & McLain Eq. 6.1)
    %   φ(s)/δa(s) = a_φ2 / [s(s + a_φ1)]
    %   Control:  δa = kp_φ*(φ_c − φ) + kd_φ*p + δa_trim
    %   Char poly: s² + (a_φ1 − a_φ2·kd_φ)s + a_φ2·kp_φ = 0
    %   → kp_φ = ωn² / a_φ2
    %   → kd_φ = (a_φ1 − 2ζ·ωn) / a_φ2
    % -----------------------------------------------------------------------
    gains.kp_phi = wn_phi^2 / tf_s.a_phi2;
    gains.kd_phi = (tf_s.a_phi1 - 2*zeta_phi*wn_phi) / tf_s.a_phi2;
    gains.ki_phi = 0.0;          % integral usually not needed for roll
    gains.wn_phi   = wn_phi;
    gains.zeta_phi = zeta_phi;

    % -----------------------------------------------------------------------
    % PITCH CHANNEL  (Beard & McLain Eq. 6.3)
    %   θ(s)/δe(s) = a_θ3 / (s² + a_θ1·s + a_θ2)
    %   Control:  δe = kp_θ*(θ_c − θ) + kd_θ*q + δe_trim
    %   Char poly: s² + (a_θ1 − a_θ3·kd_θ)s + (a_θ2 − a_θ3·kp_θ) = 0
    %   → kp_θ = (a_θ2 − ωn²) / a_θ3     (negative for ωn < √a_θ2)
    %   → kd_θ = (a_θ1 − 2ζ·ωn) / a_θ3  (positive when a_θ3 < 0)
    %
    %   NOTE: a_θ3 < 0 so kp_θ < 0 for typical autopilot BW (ωn < 10 rad/s)
    %   This is CORRECT: negative elevator when pitch error is positive.
    % -----------------------------------------------------------------------
    gains.kp_theta = (tf_s.a_theta2 - wn_theta^2) / tf_s.a_theta3;
    gains.kd_theta = (tf_s.a_theta1 - 2*zeta_theta*wn_theta) / tf_s.a_theta3;
    gains.ki_theta = 0.0;
    gains.wn_theta   = wn_theta;
    gains.zeta_theta = zeta_theta;

    % -----------------------------------------------------------------------
    % AIRSPEED CHANNEL  (Beard & McLain Eq. 6.7)
    %   Va(s)/δt(s) = a_V2 / (s + a_V1)
    %   Control:  δt = δt_trim + kp_V*(Va_c − Va) + ki_V*∫(Va_c − Va)
    %   Char poly: s² + (a_V1 + a_V2·kp_V)s + a_V2·ki_V = 0
    %   → kp_V = (2ζ·ωn − a_V1) / a_V2
    %   → ki_V = ωn² / a_V2
    % -----------------------------------------------------------------------
    gains.kp_V = (2*zeta_V*wn_V - tf_s.a_V1) / tf_s.a_V2;
    gains.ki_V = wn_V^2 / tf_s.a_V2;
    gains.wn_V   = wn_V;
    gains.zeta_V = zeta_V;

    % -----------------------------------------------------------------------
    % ALTITUDE CHANNEL  (Beard & McLain Eq. 6.9-6.10)
    %   Simplified: ḣ ≈ Va*(θ − α*)
    %   Control:  θ_c = θ_trim + kp_h*(h_c − h)
    %   First-order response: τ_h = 1/(Va*kp_h) → kp_h = ωn_h/Va*
    %   Zone logic: if |e_h| > h_zone, clip θ_c to θ_max
    % -----------------------------------------------------------------------
    gains.kp_h      = wn_h / tf_s.Va_star;    % [rad/m]
    gains.ki_h      = 0.0;
    gains.wn_h      = wn_h;
    gains.h_zone    = 10.0;                    % [m]  proportional zone ±10m
    gains.theta_max =  deg2rad(30);            % [rad] max climb pitch
    gains.theta_min = -deg2rad(20);            % [rad] max descent pitch

    % -----------------------------------------------------------------------
    % COURSE CHANNEL  (Beard & McLain Eq. 6.11)
    %   Simplified: χ̇ ≈ (g/Va)*φ   (level flight, coordinated turn)
    %   Control:  φ_c = kp_χ*(χ_c − χ)
    %   First-order: kp_χ = ωn_χ * Va* / g
    % -----------------------------------------------------------------------
    gains.kp_chi  = wn_chi * tf_s.Va_star / params.gravity;   % [rad/rad]
    gains.ki_chi  = 0.0;
    gains.wn_chi  = wn_chi;
    gains.phi_max = deg2rad(45);               % [rad] max bank angle

    % -----------------------------------------------------------------------
    % STORE TRIM REFERENCE (needed in each controller for feedforward)
    % -----------------------------------------------------------------------
    gains.Va_star     = tf_s.Va_star;
    gains.alpha_star  = tf_s.alpha_star;
    gains.theta_star  = tf_s.theta_star;
    gains.delta_e_star = tf_s.delta_e_star;
    gains.delta_t_star = tf_s.delta_t_star;

end
