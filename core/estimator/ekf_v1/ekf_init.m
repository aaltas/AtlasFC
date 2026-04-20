% =========================================================================
%  EKF_INIT - Initialize 13-state INS EKF
% =========================================================================
%  Returns the initial state estimate and covariance for the EKF.
%
%  Inputs:
%    x13_init  [13x1] true (or best-guess) initial state
%    ep        struct from ekf_params()
%
%  Outputs:
%    x_hat     [13x1] initial state estimate (= x13_init)
%    P         [13x13] initial covariance (= ep.P0)
%
%  Author : AtlasFC  |  Ref: Beard & McLain Ch. 8
% =========================================================================

function [x_hat, P] = ekf_init(x13_init, ep)

    x_hat = x13_init;

    % Normalize initial quaternion
    x_hat(7:10) = x_hat(7:10) / norm(x_hat(7:10));

    P = ep.P0;

end
