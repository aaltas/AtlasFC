% =========================================================================
%  EKF  —  v2 placeholder
% =========================================================================
%  Replace this file with your new EKF implementation.
%
%  REQUIRED INTERFACE (must match ekf_v1/ekf.m exactly):
%
%    [x_hat, P, innov] = ekf(x_hat, P, y, ep, params, dt)
%
%  Inputs / Outputs: identical to ekf_v1 — see ekf_v1/ekf.m for full spec.
%
%  Files to implement in this folder:
%    ekf.m            ← this file  (top-level call, same signature)
%    ekf_propagate.m  ← prediction step
%    ekf_update.m     ← correction step
%    ekf_init.m       ← state + covariance initialization
%    ekf_params.m     ← Q, R, P0  tuned for this version
%    nav_equations.m  ← keep or replace with new state model
%
%  The selector in atlas_sim.m / run_flight_eval.m adds THIS folder to
%  the MATLAB path when cfg.ekf_version = 'ekf_v2'.  No other changes
%  needed anywhere.
%
%  STATUS: TODO
% =========================================================================

function [x_hat, P, innov] = ekf(x_hat, P, y, ep, params, dt)
    error('ekf_v2 is not yet implemented. See ekf_v2/ekf.m for instructions.');
end
