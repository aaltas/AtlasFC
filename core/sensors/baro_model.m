% =========================================================================
%  BARO_MODEL - Barometric altimeter sensor model
% =========================================================================
%  Models a barometric pressure altimeter with:
%    - Gauss-Markov bias (slowly time-varying pressure offset)
%    - White Gaussian measurement noise
%
%  Measurement equation:
%    y_baro = h + bias + noise
%    where h = -pd  (altitude positive up, pd positive down in NED)
%
%  Bias dynamics (discrete Gauss-Markov):
%    b_k+1 = exp(-dt/tau) * b_k + sigma_b * sqrt(1-exp(-2*dt/tau)) * w
%
%  Inputs:
%    x13       - 13-state vector [pn,pe,pd,u,v,w,e0,e1,e2,e3,p,q,r]
%    bias      - [scalar] current bias state [m]
%    sparams   - sensor_params() struct
%    dt        - time step [s]
%
%  Outputs:
%    y_baro    - [scalar] measured altitude [m]
%    bias_new  - [scalar] updated bias state [m]
%
%  Usage:
%    [y_baro, baro_bias] = baro_model(x13, baro_bias, sparams, dt);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 7
% =========================================================================

function [y_baro, bias_new] = baro_model(x13, bias, sparams, dt)

    % True altitude (NED: pd positive down → h = -pd)
    h_true = -x13(3);

    % --- Gauss-Markov bias propagation ---
    alpha   = exp(-dt / sparams.tau_baro);
    sigma_d = sparams.sigma_b_baro * sqrt(1 - alpha^2);

    bias_new = alpha * bias + sigma_d * randn;

    % --- Measurement: truth + bias + white noise ---
    noise  = sparams.sigma_baro * randn;

    y_baro = h_true + bias_new + noise;

end
