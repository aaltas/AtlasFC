% =========================================================================
%  MEAS_GPS - GPS measurement model for EKF (4-channel)
% =========================================================================
%  Measurement vector (4×1):
%    z_gps = [pn_meas; pe_meas; Vg_meas; chi_meas]
%
%  Predicted measurement h(x):
%    h(1) = pn               =  x(1)
%    h(2) = pe               =  x(2)
%    h(3) = Vg               =  sqrt(u² + v² + w²)       (zero-wind)
%    h(4) = chi (course)     =  atan2(v_NED_E, v_NED_N)  (zero-wind)
%
%  For zero wind: Vg = Va, chi = heading extracted from velocity direction
%  in NED frame, where [v_NED_N; v_NED_E] = R_vb(1:2, :) * v_body.
%
%  Jacobian H = ∂h/∂x  (4×13):
%
%    Row 1 (pn):  H = [1, 0, 0, 0, ..., 0]
%    Row 2 (pe):  H = [0, 1, 0, 0, ..., 0]
%
%    Row 3 (Vg):  H(4:6) = [u, v, w] / Vg    (same as pitot)
%                 H(else) = 0
%
%    Row 4 (chi): ∂chi/∂v  = (1/D) * [vN*R_vb(2,:) - vE*R_vb(1,:)]  (1×3)
%                 ∂chi/∂q  = (1/D) * [vN*∂vE/∂q   - vE*∂vN/∂q  ]   (1×4)
%                 D = vN² + vE²
%
%    where ∂vN/∂q and ∂vE/∂q come from the first two rows of the
%    position-quaternion Jacobian  ∂ṗ/∂q  (same matrix used in A(1:3,7:10)
%    in nav_equations.m).
%
%  Innovation wrapping:
%    The chi innovation must be wrapped to (-π, π].
%    Pass wrap_idx = 4 to ekf_update.
%
%  Inputs:
%    x_hat [13x1] current state estimate
%
%  Outputs:
%    h_pred [4x1]  predicted GPS measurement [pn; pe; Vg; chi]
%    H      [4x13] measurement Jacobian
%
%  Author : AtlasFC  |  Ref: Beard & McLain Ch. 8
% =========================================================================

function [h_pred, H] = meas_gps(x_hat)

    pn = x_hat(1);  pe = x_hat(2);
    u_ = x_hat(4);  v_ = x_hat(5);  w_ = x_hat(6);
    e0 = x_hat(7);  e1 = x_hat(8);  e2 = x_hat(9);  e3 = x_hat(10);

    % Rotation matrix NED←body
    R_bv = quaternion_to_rotation([e0; e1; e2; e3]);
    R_vb = R_bv';

    vb = [u_; v_; w_];

    % NED velocity (zero wind)
    v_NED = R_vb * vb;
    vN    = v_NED(1);
    vE    = v_NED(2);

    % Airspeed / ground speed
    Va = sqrt(u_^2 + v_^2 + w_^2);

    % Course angle
    chi = atan2(vE, vN);

    h_pred = [pn; pe; Va; chi];

    H = zeros(4, 13);

    % --- Row 1: pn ---
    H(1, 1) = 1;

    % --- Row 2: pe ---
    H(2, 2) = 1;

    % --- Row 3: Vg = Va ---
    if Va > 1e-3
        H(3, 4) = u_ / Va;
        H(3, 5) = v_ / Va;
        H(3, 6) = w_ / Va;
    end

    % --- Row 4: chi = atan2(vE, vN) ---
    D = vN^2 + vE^2;
    if D > 1e-6
        % ∂chi/∂v_body = (1/D) * [vN*R_vb(2,:) - vE*R_vb(1,:)]
        H(4, 4:6) = (1/D) * (vN * R_vb(2,:) - vE * R_vb(1,:));

        % ∂chi/∂q: need first two rows of ∂(R_vb*vb)/∂q
        % = (1/D) * [vN * ∂vE/∂q - vE * ∂vN/∂q]
        % ∂vN/∂q = row 1 of dp_dq,  ∂vE/∂q = row 2 of dp_dq
        dp_dq = 2 * [...
            e0*u_-e3*v_+e2*w_,  e1*u_+e2*v_+e3*w_, -e2*u_+e1*v_+e0*w_, -e3*u_-e0*v_+e1*w_;
            e3*u_+e0*v_-e1*w_,  e2*u_-e1*v_-e0*w_,  e1*u_+e2*v_+e3*w_,  e0*u_-e3*v_+e2*w_];

        dVn_dq = dp_dq(1, :);   % 1×4
        dVe_dq = dp_dq(2, :);   % 1×4

        H(4, 7:10) = (1/D) * (vN * dVe_dq - vE * dVn_dq);
    end

end
