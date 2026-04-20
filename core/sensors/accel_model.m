% =========================================================================
%  ACCEL_MODEL - Accelerometer (specific force) sensor model
% =========================================================================
%  Models a 3-axis MEMS accelerometer with:
%    - Gauss-Markov bias
%    - White Gaussian measurement noise
%
%  The accelerometer measures SPECIFIC FORCE (not total acceleration):
%    f_body = F_aero+prop / mass  +  noise
%
%  Note: gravity does NOT appear directly in the specific force because
%  the accelerometer is in free fall with the aircraft.  However, when
%  the aircraft is in level flight, the structural reaction to lift means
%  the sensor reads the non-gravitational forces.  In the body frame:
%
%    y_accel = fm(1:3)/mass - R_vb^T * [0; 0; g]  + bias + noise
%
%  where R_vb is the vehicle-to-body rotation (R^T_{bv}) so that
%  R_vb^T = R_bv converts the gravity NED vector to body frame.
%  Using Beard & McLain sign convention: positive pd is down, so
%  gravity in NED = [0; 0; +g], and in body frame = R_bv * [0;0;g].
%
%  Bias dynamics (discrete Gauss-Markov):
%    b_k+1 = exp(-dt/tau) * b_k + sigma_b * sqrt(1-exp(-2*dt/tau)) * w
%
%  Inputs:
%    x13       - 13-state vector [pn,pe,pd,u,v,w,e0,e1,e2,e3,p,q,r]
%    fm        - [6x1] forces+moments [Fx,Fy,Fz,Mx,My,Mz] [N, N*m]
%    bias      - [3x1] current bias state [m/s^2]
%    sparams   - sensor_params() struct
%    params    - mav_params() struct (needs mass, gravity)
%    dt        - time step [s]
%
%  Outputs:
%    y_accel   - [3x1] measured specific force [fx_m; fy_m; fz_m] [m/s^2]
%    bias_new  - [3x1] updated bias state [m/s^2]
%
%  Usage:
%    [y_accel, accel_bias] = accel_model(x13, fm, accel_bias, sparams, params, dt);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 7
% =========================================================================

function [y_accel, bias_new] = accel_model(x13, fm, bias, sparams, params, dt)

    % --- True specific force (body frame) ---
    % Force from aerodynamics + propulsion, divided by mass
    f_specific = fm(1:3) / params.mass;

    % Subtract gravity component projected into body frame
    % Rotation matrix from quaternion
    R_bv = quaternion_to_rotation(x13(7:10));  % body←vehicle (NED)
    g_body = R_bv * [0; 0; params.gravity];    % gravity in body frame

    % Specific force = non-gravitational acceleration = f_aero/m - g_body
    % (accelerometer reads f_aero/m - g in body because gravity is "free fall")
    % Standard B&M convention:
    f_true = f_specific - g_body;

    % --- Gauss-Markov bias propagation ---
    alpha   = exp(-dt / sparams.tau_accel);
    sigma_d = sparams.sigma_b_accel * sqrt(1 - alpha^2);

    bias_new = alpha * bias + sigma_d * randn(3, 1);

    % --- Measurement: truth + bias + white noise ---
    noise   = sparams.sigma_accel * randn(3, 1);

    y_accel = f_true + bias_new + noise;

end
