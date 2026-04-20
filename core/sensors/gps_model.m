% =========================================================================
%  GPS_MODEL - GPS position and velocity sensor model
% =========================================================================
%  Models a GPS receiver providing position (pn, pe) and velocity
%  (ground speed Vg, course angle chi) at a low update rate (typ. 1 Hz).
%
%  The GPS model uses Gauss-Markov processes for each output to model
%  spatially correlated position errors (multipath, atmospheric).
%
%  Measurement equations:
%    y_pn  = pn + b_pn + noise_pn
%    y_pe  = pe + b_pe + noise_pe
%    y_Vg  = Vg + b_Vg + noise_Vg      (Vg = ground speed, zero wind → Va)
%    y_chi = chi + b_chi + noise_chi   (chi = course, zero wind → psi)
%
%  Note: GPS does NOT measure altitude reliably for small UAVs — barometer
%  is used instead (as in B&M Ch. 7-8).
%
%  GPS timer logic:
%    GPS output is only updated at 1/gps_hz intervals.
%    Between updates, the last GPS reading is held (zero-order hold).
%    The gps_timer in sensor_state tracks when the next update fires.
%
%  Bias dynamics (discrete Gauss-Markov, very slow tau → quasi-static):
%    b_k+1 = exp(-dt/tau) * b_k + sigma_b * sqrt(1-exp(-2*dt/tau)) * w
%
%  Inputs:
%    x13          - 13-state vector [pn,pe,pd,u,v,w,e0,e1,e2,e3,p,q,r]
%    Va_true      - [scalar] true airspeed [m/s] (= Vg for zero wind)
%    gps_bias     - [4x1] current GPS bias [b_pn; b_pe; b_Vg; b_chi]
%    sparams      - sensor_params() struct
%    dt           - simulation time step [s]
%    gps_timer    - [scalar] countdown timer to next GPS update [s]
%    last_gps     - [4x1] last GPS output [pn_m; pe_m; Vg_m; chi_m]
%
%  Outputs:
%    y_gps        - [4x1] GPS measurement [pn_m; pe_m; Vg_m; chi_m]
%                   (held at last value between updates)
%    gps_bias_new - [4x1] updated bias state
%    gps_timer_new- [scalar] updated timer [s]
%    last_gps_new - [4x1] updated held GPS output
%
%  Usage (inside sensor loop):
%    [y_gps, gps_bias, gps_timer, last_gps] = ...
%        gps_model(x13, Va, gps_bias, sparams, dt, gps_timer, last_gps);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 7
% =========================================================================

function [y_gps, gps_bias_new, gps_timer_new, last_gps_new] = ...
         gps_model(x13, Va_true, gps_bias, sparams, dt, gps_timer, last_gps)

    % --- Gauss-Markov bias propagation (runs every IMU step) ---
    tau_vec   = [sparams.tau_gps_pn; sparams.tau_gps_pe;
                 sparams.tau_gps_Vg; sparams.tau_gps_chi];
    sig_b_vec = [sparams.sigma_b_gps_pn; sparams.sigma_b_gps_pe;
                 sparams.sigma_b_gps_Vg; sparams.sigma_b_gps_chi];

    alpha_vec = exp(-dt ./ tau_vec);
    sigma_d   = sig_b_vec .* sqrt(1 - alpha_vec.^2);

    gps_bias_new = alpha_vec .* gps_bias + sigma_d .* randn(4, 1);

    % --- Decrement GPS timer ---
    gps_timer_new = gps_timer - dt;

    if gps_timer_new <= 0
        % ---- GPS fires: generate new measurement ----

        % True states
        pn_true  = x13(1);
        pe_true  = x13(2);

        % Ground speed ≈ Va for zero-wind case
        Vg_true  = Va_true;

        % Course angle ≈ heading for zero-wind case
        R_bv     = quaternion_to_rotation(x13(7:10));
        [~, ~, psi_true] = rotation_to_euler(R_bv);
        chi_true = psi_true;

        % White noise on GPS output
        sig_vec = [sparams.sigma_gps_pn; sparams.sigma_gps_pe;
                   sparams.sigma_gps_Vg; sparams.sigma_gps_chi];
        noise   = sig_vec .* randn(4, 1);

        % GPS measurement = truth + bias + noise
        true_vec   = [pn_true; pe_true; Vg_true; chi_true];
        last_gps_new = true_vec + gps_bias_new + noise;

        % Wrap course angle
        last_gps_new(4) = atan2(sin(last_gps_new(4)), cos(last_gps_new(4)));

        % Reset timer
        gps_timer_new = 1.0 / sparams.gps_hz;

    else
        % GPS not yet updated — hold last measurement
        last_gps_new = last_gps;
    end

    y_gps = last_gps_new;

end
