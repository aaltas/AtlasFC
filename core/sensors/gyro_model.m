% =========================================================================
%  GYRO_MODEL - Rate gyroscope sensor model
% =========================================================================
%  Models a 3-axis MEMS rate gyroscope with:
%    - Gauss-Markov bias (slowly time-varying)
%    - White Gaussian measurement noise
%
%  Measurement equation (body frame):
%    y_gyro = [p; q; r] + bias + noise
%
%  Bias dynamics (discrete Gauss-Markov):
%    b_k+1 = exp(-dt/tau) * b_k + sigma_b * sqrt(1 - exp(-2*dt/tau)) * w
%    where w ~ N(0,I)
%
%  Inputs:
%    x13       - 13-state vector [pn,pe,pd,u,v,w,e0,e1,e2,e3,p,q,r]
%    bias      - [3x1] current bias state [b_p; b_q; b_r] [rad/s]
%    sparams   - sensor_params() struct
%    dt        - time step [s]
%
%  Outputs:
%    y_gyro    - [3x1] measured angular rates [p_m; q_m; r_m] [rad/s]
%    bias_new  - [3x1] updated bias state [rad/s]
%
%  Usage:
%    [y_gyro, bias] = gyro_model(x13, bias, sparams, dt);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 7
% =========================================================================

function [y_gyro, bias_new] = gyro_model(x13, bias, sparams, dt)

    % True angular rates from state vector
    p_true = x13(11);
    q_true = x13(12);
    r_true = x13(13);

    % --- Gauss-Markov bias propagation ---
    alpha   = exp(-dt / sparams.tau_gyro);
    sigma_d = sparams.sigma_b_gyro * sqrt(1 - alpha^2);

    bias_new = alpha * bias + sigma_d * randn(3, 1);

    % --- Measurement: truth + bias + white noise ---
    noise  = sparams.sigma_gyro * randn(3, 1);

    y_gyro = [p_true; q_true; r_true] + bias_new + noise;

end
