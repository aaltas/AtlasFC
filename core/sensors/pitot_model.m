% =========================================================================
%  PITOT_MODEL - Pitot tube airspeed sensor model
% =========================================================================
%  Models a pitot-static airspeed sensor with:
%    - Gauss-Markov bias (slowly time-varying offset)
%    - White Gaussian measurement noise
%    - One-sided saturation: airspeed cannot be negative
%
%  Measurement equation:
%    y_pitot = Va + bias + noise,  y_pitot >= 0
%    where Va = sqrt(ur^2 + vr^2 + wr^2)  (relative/air-mass airspeed)
%    In zero-wind case: Va = sqrt(u^2 + v^2 + w^2)
%
%  Bias dynamics (discrete Gauss-Markov):
%    b_k+1 = exp(-dt/tau) * b_k + sigma_b * sqrt(1-exp(-2*dt/tau)) * w
%
%  Inputs:
%    x13       - 13-state vector [pn,pe,pd,u,v,w,e0,e1,e2,e3,p,q,r]
%    Va_wind   - [scalar] wind-relative airspeed [m/s]
%                (pass Va_true directly; computed externally from wind)
%    bias      - [scalar] current bias state [m/s]
%    sparams   - sensor_params() struct
%    dt        - time step [s]
%
%  Outputs:
%    y_pitot   - [scalar] measured airspeed [m/s], clipped to >= 0
%    bias_new  - [scalar] updated bias state [m/s]
%
%  Usage:
%    Va_true = sqrt(x13(4)^2 + x13(5)^2 + x13(6)^2);  % zero-wind case
%    [y_pitot, pitot_bias] = pitot_model(x13, Va_true, pitot_bias, sparams, dt);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 7
% =========================================================================

function [y_pitot, bias_new] = pitot_model(x13, Va_true, bias, sparams, dt)

    % --- Gauss-Markov bias propagation ---
    alpha   = exp(-dt / sparams.tau_pitot);
    sigma_d = sparams.sigma_b_pitot * sqrt(1 - alpha^2);

    bias_new = alpha * bias + sigma_d * randn;

    % --- Measurement: truth + bias + white noise ---
    noise    = sparams.sigma_pitot * randn;

    y_pitot  = Va_true + bias_new + noise;

    % Airspeed cannot be negative (physical lower bound)
    y_pitot  = max(0, y_pitot);

end
