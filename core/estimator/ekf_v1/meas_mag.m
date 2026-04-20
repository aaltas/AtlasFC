% =========================================================================
%  MEAS_MAG - Magnetometer (heading) measurement model for EKF
% =========================================================================
%  Measurement function (zero declination):
%    m_body = R_bv * [1; 0; 0]  =  first column of R_bv
%    h(x)   = atan2(m_body(2), m_body(1))
%
%  In terms of quaternion q = [e0; e1; e2; e3]:
%    m_x = e0² + e1² - e2² - e3²
%    m_y = 2*(e1*e2 - e0*e3)
%    h   = atan2(m_y, m_x)
%
%  Jacobian H = ∂h/∂x  (1×13):
%    H(7:10) = ∂(atan2(m_y, m_x))/∂q   (chain rule through atan2)
%    All other columns = 0
%
%    ∂h/∂q = [m_x*∂m_y/∂q - m_y*∂m_x/∂q] / (m_x² + m_y²)
%
%    where:
%      ∂m_x/∂q = [2e0,  2e1, -2e2, -2e3]
%      ∂m_y/∂q = [-2e3, 2e2,  2e1, -2e0]
%
%  Innovation wrapping:
%    The heading innovation must be wrapped to (-π, π].
%    Pass wrap_idx = 1 to ekf_update.
%
%  Singularity:
%    If m_x²+m_y² < 1e-6 (aircraft nearly vertical), H = 0.
%
%  Inputs:
%    x_hat [13x1] current state estimate
%
%  Outputs:
%    h_pred [1x1]  predicted heading [rad], wrapped to (-π, π]
%    H      [1x13] measurement Jacobian
%
%  Author : AtlasFC  |  Ref: Beard & McLain Ch. 8
% =========================================================================

function [h_pred, H] = meas_mag(x_hat)

    e0 = x_hat(7);  e1 = x_hat(8);
    e2 = x_hat(9);  e3 = x_hat(10);

    % Magnetic field vector in body frame (first column of R_bv)
    m_x = e0^2 + e1^2 - e2^2 - e3^2;
    m_y = 2*(e1*e2 - e0*e3);

    h_pred = atan2(m_y, m_x);

    H = zeros(1, 13);

    denom = m_x^2 + m_y^2;
    if denom > 1e-6
        % ∂m_x/∂q and ∂m_y/∂q  (each 1×4)
        dm_x = 2*[e0,  e1, -e2, -e3];
        dm_y = 2*[-e3, e2,  e1, -e0];

        % Chain rule: ∂atan2(m_y,m_x)/∂q
        H(1, 7:10) = (m_x * dm_y - m_y * dm_x) / denom;
    end

end
