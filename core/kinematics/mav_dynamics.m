% =========================================================================
%  MAV_DYNAMICS - 6-DOF Equations of Motion with RK4 Integration
% =========================================================================
%  Propagates the 13-state MAV state vector one time step forward using
%  Runge-Kutta 4th order (RK4) integration.
%
%  State vector (13 x 1):
%    state(1)  = pn   - North position [m]
%    state(2)  = pe   - East  position [m]
%    state(3)  = pd   - Down  position [m]  (positive = below ground)
%    state(4)  = u    - body-frame x velocity [m/s]
%    state(5)  = v    - body-frame y velocity [m/s]
%    state(6)  = w    - body-frame z velocity [m/s]
%    state(7)  = e0   - quaternion scalar
%    state(8)  = e1   - quaternion x
%    state(9)  = e2   - quaternion y
%    state(10) = e3   - quaternion z
%    state(11) = p    - roll  rate [rad/s]
%    state(12) = q    - pitch rate [rad/s]
%    state(13) = r    - yaw   rate [rad/s]
%
%  Forces & Moments input (6 x 1):
%    fm(1) = fx  - total force  in body x [N]
%    fm(2) = fy  - total force  in body y [N]
%    fm(3) = fz  - total force  in body z [N]
%    fm(4) = l   - rolling  moment [N*m]
%    fm(5) = m   - pitching moment [N*m]
%    fm(6) = n   - yawing   moment [N*m]
%
%  Inputs:
%    state  - [13x1] current state vector
%    fm     - [6x1]  forces and moments in body frame
%    params - struct from mav_params()
%    dt     - time step [s]
%
%  Outputs:
%    state_new - [13x1] propagated state vector
%
%  Usage:
%    state_new = mav_dynamics(state, fm, params, dt)
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 3
% =========================================================================

function state_new = mav_dynamics(state, fm, params, dt)

    % --- RK4 Integration ---
    k1 = mav_derivatives(state,           fm, params);
    k2 = mav_derivatives(state + dt/2*k1, fm, params);
    k3 = mav_derivatives(state + dt/2*k2, fm, params);
    k4 = mav_derivatives(state + dt*k3,   fm, params);

    state_new = state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

    % --- Quaternion Renormalization ---
    % Numerical drift will slowly corrupt the unit-norm constraint; fix it.
    q_norm = norm(state_new(7:10));
    state_new(7:10) = state_new(7:10) / q_norm;

end
