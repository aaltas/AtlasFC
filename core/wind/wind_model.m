% =========================================================================
%  WIND_MODEL - Steady-state wind + Dryden gust model
% =========================================================================
%  Computes the total wind vector split into:
%    - Steady-state wind in NED inertial frame [m/s]
%    - Gust wind in body frame [m/s]  (Dryden turbulence model)
%
%  The Dryden model (Slide 40) is implemented as a discrete-time state-space
%  system driven by zero-mean unit-variance Gaussian white noise.
%
%  Transfer functions (Slide 40):
%    H_u(s) = sigma_u * sqrt(2*Va/(pi*Lu)) * 1/(s + Va/Lu)
%    H_v(s) = sigma_v * sqrt(3*Va/(pi*Lv)) * (s + Va/(sqrt(3)*Lv)) / (s + Va/Lv)^2
%    H_w(s) = sigma_w * sqrt(3*Va/(pi*Lw)) * (s + Va/(sqrt(3)*Lw)) / (s + Va/Lw)^2
%
%  Discrete implementation (Slide 41):
%    ẋ = Ax + Bu  →  x_{k+1} = (I + Ts*A)*x_k + Ts*B*u_k
%    y_k = C * x_k
%
%  Inputs:
%    Va         - current airspeed [m/s] (for Dryden model)
%    dt         - time step [s]
%    wind_state - [6x1] internal Dryden filter states [x_u; x_v1; x_v2; x_w1; x_w2; unused]
%                 Initialize to zeros(6,1)
%    wind_ss    - [3x1] steady wind in NED [w_n; w_e; w_d] [m/s]
%    gust_on    - logical: true = enable Dryden gust, false = zero gust
%    turb_level - turbulence level: 'light', 'moderate' (default 'light')
%
%  Outputs:
%    wind_i     - [3x1] steady-state wind in NED inertial frame [m/s]
%    wind_b     - [3x1] gust wind in body frame [m/s]
%    wind_state_new - [6x1] updated Dryden filter states
%
%  Usage (no gust):
%    [wind_i, wind_b, ws] = wind_model(Va, dt, zeros(6,1), [5;0;0], false, 'light')
%
%  Usage (with gust):
%    [wind_i, wind_b, ws] = wind_model(Va, dt, ws, wind_ss, true, 'light')
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 4, Slides 38-41
% =========================================================================

function [wind_i, wind_b, wind_state_new] = wind_model(Va, dt, wind_state, ...
                                                         wind_ss, gust_on, turb_level)

    % --- Steady wind (directly in NED) ---
    wind_i = wind_ss;   % [3x1]

    % Default: no gust
    wind_b = zeros(3,1);
    wind_state_new = wind_state;

    if ~gust_on
        return;
    end

    % --- Dryden turbulence parameters (Slide 40, Table 1) ---
    if strcmp(turb_level, 'moderate')
        sigma_u = 2.12;  sigma_v = 2.12;  sigma_w = 1.4;
        Lu = 200;        Lv = 200;        Lw = 50;
    else  % 'light' (default)
        sigma_u = 1.06;  sigma_v = 1.06;  sigma_w = 0.7;
        Lu = 200;        Lv = 200;        Lw = 50;
    end

    Va = max(Va, 1.0);   % avoid divide-by-zero

    % --- Unpack filter states ---
    x_u  = wind_state(1);       % 1-state for u_gust filter
    x_v1 = wind_state(2);       % 2-state for v_gust filter
    x_v2 = wind_state(3);
    x_w1 = wind_state(4);       % 2-state for w_gust filter
    x_w2 = wind_state(5);

    % --- White noise inputs (unit variance Gaussian) ---
    n_u = randn();
    n_v = randn();
    n_w = randn();

    % --- H_u: first-order system  Y(s) = sigma_u*sqrt(2Va/(pi*Lu)) / (s+Va/Lu) ---
    % Continuous: ẋ_u = -(Va/Lu)*x_u + n_u;  u_gust = sigma_u*sqrt(2Va/(pi*Lu))*x_u
    Au = -(Va / Lu);
    Bu = 1.0;
    x_u_new = x_u + dt * (Au * x_u + Bu * n_u);
    u_gust  = sigma_u * sqrt(2*Va / (pi * Lu)) * x_u_new;

    % --- H_v: second-order system ---
    % Continuous: ẋ = [-2Va/Lv, -(Va/Lv)^2; 1, 0]*x + [1;0]*n
    %             v_gust = sigma_v*sqrt(3Va/(pi*Lv)) * [Va/(sqrt(3)*Lv), 1]*x
    Av = [-(2*Va/Lv),  -(Va/Lv)^2; 1, 0];
    Bv = [1; 0];
    Cv = sigma_v * sqrt(3*Va/(pi*Lv)) * [Va/(sqrt(3)*Lv), 1];
    xv     = [x_v1; x_v2];
    xv_new = xv + dt * (Av * xv + Bv * n_v);
    v_gust = Cv * xv_new;

    % --- H_w: second-order system (same structure as H_v) ---
    Aw = [-(2*Va/Lw),  -(Va/Lw)^2; 1, 0];
    Bw = [1; 0];
    Cw = sigma_w * sqrt(3*Va/(pi*Lw)) * [Va/(sqrt(3)*Lw), 1];
    xw     = [x_w1; x_w2];
    xw_new = xw + dt * (Aw * xw + Bw * n_w);
    w_gust = Cw * xw_new;

    % --- Pack gust and updated states ---
    wind_b = [u_gust; v_gust; w_gust];

    wind_state_new = [x_u_new; xv_new(1); xv_new(2); xw_new(1); xw_new(2); 0];

end
