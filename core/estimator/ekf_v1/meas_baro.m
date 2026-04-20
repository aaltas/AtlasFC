% =========================================================================
%  MEAS_BARO - Barometer measurement model for EKF
% =========================================================================
%  Measurement function:
%    h(x) = -pd = -x(3)      (altitude = -down position)
%
%  Jacobian H = ∂h/∂x  (1×13):
%    H = [0, 0, -1, 0, ..., 0]
%
%  Inputs:
%    x_hat [13x1] current state estimate
%
%  Outputs:
%    h_pred [1x1]  predicted barometer reading [m]
%    H      [1x13] measurement Jacobian
%
%  Usage:
%    [h_pred, H] = meas_baro(x_hat);
%    [x_hat, P, innov] = ekf_update(x_hat, P, H, ep.R_baro, y.baro, h_pred);
%
%  Author : AtlasFC  |  Ref: Beard & McLain Ch. 8
% =========================================================================

function [h_pred, H] = meas_baro(x_hat)

    % Predicted altitude from state (h = -pd)
    h_pred = -x_hat(3);

    % Jacobian: ∂(-pd)/∂x  →  only ∂/∂pd = -1  (index 3)
    H = zeros(1, 13);
    H(1, 3) = -1;

end
