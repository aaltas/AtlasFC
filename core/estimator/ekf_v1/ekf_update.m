% =========================================================================
%  EKF_UPDATE - Generic EKF measurement update step
% =========================================================================
%  Applies one measurement update given a predicted measurement h(x),
%  the measurement Jacobian H = ∂h/∂x, measurement noise R, and the
%  actual measurement z.
%
%  UPDATE EQUATIONS:
%    S      = H * P * H' + R          (innovation covariance)
%    K      = P * H' * inv(S)         (Kalman gain)
%    innov  = z - h_pred              (innovation / residual)
%    x_hat  = x_hat + K * innov       (state correction)
%    P      = (I - K*H) * P           (Joseph form for stability)
%
%  ANGLE WRAPPING:
%    For angle measurements (heading, course), the innovation must be
%    wrapped to (-π, π] to avoid 2π jumps.  Pass the indices of angle
%    measurements in wrap_idx.
%
%  QUATERNION RENORMALIZATION:
%    After the state correction, the quaternion x_hat(7:10) is
%    renormalized.  This is mandatory — the EKF update does not enforce
%    the unit-norm constraint.
%
%  Inputs:
%    x_hat    [nx1]  current state estimate
%    P        [nxn]  current covariance
%    H        [mxn]  measurement Jacobian ∂h/∂x
%    R        [mxm]  measurement noise covariance
%    z        [mx1]  actual measurement
%    h_pred   [mx1]  predicted measurement h(x_hat)
%    wrap_idx (optional) vector of measurement indices to angle-wrap
%
%  Outputs:
%    x_hat    [nx1]  updated state estimate
%    P        [nxn]  updated covariance
%    innov    [mx1]  innovation vector (for logging/analysis)
%
%  Author : AtlasFC  |  Ref: Beard & McLain Ch. 8
% =========================================================================

function [x_hat, P, innov] = ekf_update(x_hat, P, H, R, z, h_pred, wrap_idx)

    % -----------------------------------------------------------------------
    %  1. Innovation
    % -----------------------------------------------------------------------
    innov = z - h_pred;

    % Angle wrapping for specified indices
    if nargin >= 7 && ~isempty(wrap_idx)
        innov(wrap_idx) = atan2(sin(innov(wrap_idx)), cos(innov(wrap_idx)));
    end

    % -----------------------------------------------------------------------
    %  2. Innovation covariance  S = H*P*H' + R
    % -----------------------------------------------------------------------
    S = H * P * H' + R;

    % -----------------------------------------------------------------------
    %  3. Kalman gain  K = P*H'*inv(S)
    % -----------------------------------------------------------------------
    K = P * H' / S;

    % -----------------------------------------------------------------------
    %  4. State update
    % -----------------------------------------------------------------------
    x_hat = x_hat + K * innov;

    % -----------------------------------------------------------------------
    %  5. Covariance update (Joseph form for numerical stability)
    %     P = (I - K*H) * P * (I - K*H)' + K*R*K'
    % -----------------------------------------------------------------------
    n   = length(x_hat);
    IKH = eye(n) - K * H;
    P   = IKH * P * IKH' + K * R * K';
    P   = 0.5 * (P + P');   % enforce symmetry

    % -----------------------------------------------------------------------
    %  6. Re-normalize quaternion
    % -----------------------------------------------------------------------
    qn = norm(x_hat(7:10));
    if qn > 1e-10
        x_hat(7:10) = x_hat(7:10) / qn;
    end

end
