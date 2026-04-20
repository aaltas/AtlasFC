% =========================================================================
%  MEAS_PITOT - Pitot tube (airspeed) measurement model for EKF
% =========================================================================
%  Measurement function (zero-wind assumption):
%    h(x) = Va = sqrt(u² + v² + w²) = norm(x(4:6))
%
%  Jacobian H = ∂h/∂x  (1×13):
%    ∂Va/∂u = u/Va,  ∂Va/∂v = v/Va,  ∂Va/∂w = w/Va
%    All other partials = 0
%
%  Singularity protection:
%    If Va < 1e-3 m/s, H is set to zero (avoids division by near-zero Va).
%    The EKF will simply not correct on airspeed at very low speeds.
%
%  Inputs:
%    x_hat [13x1] current state estimate
%
%  Outputs:
%    h_pred [1x1]  predicted airspeed [m/s]
%    H      [1x13] measurement Jacobian
%
%  Usage:
%    [h_pred, H] = meas_pitot(x_hat);
%    [x_hat, P, innov] = ekf_update(x_hat, P, H, ep.R_pitot, y.pitot, h_pred);
%
%  Author : AtlasFC  |  Ref: Beard & McLain Ch. 8
% =========================================================================

function [h_pred, H] = meas_pitot(x_hat)

    u_ = x_hat(4);
    v_ = x_hat(5);
    w_ = x_hat(6);

    Va = sqrt(u_^2 + v_^2 + w_^2);
    h_pred = Va;

    H = zeros(1, 13);
    if Va > 1e-3
        H(1, 4) = u_ / Va;
        H(1, 5) = v_ / Va;
        H(1, 6) = w_ / Va;
    end

end
