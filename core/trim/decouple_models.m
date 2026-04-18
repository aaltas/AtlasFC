% =========================================================================
%  DECOUPLE_MODELS - Extract longitudinal and lateral state-space models
% =========================================================================
%  Partitions the full 12-state linearized model into decoupled
%  longitudinal and lateral-directional sub-models for autopilot design.
%
%  Full 12-state ordering (from linearize.m):
%    x = [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
%          1   2   3  4  5  6   7     8     9  10 11 12
%
%  LONGITUDINAL sub-model  (Slide 37, Beard & McLain Ch. 5):
%    x_lon = [u, w, q, theta, h]   (5 states)
%    u_lon = [delta_e, delta_t]    (2 inputs)
%    h = -pd  (altitude positive up)  → sign flip on pd row/column
%
%  LATERAL sub-model  (Slide 40):
%    x_lat = [v, p, r, phi, psi]   (5 states)
%    u_lat = [delta_a, delta_r]    (2 inputs)
%
%  Inputs:
%    A  [12x12] - full state Jacobian (from linearize)
%    B  [12x4]  - full input Jacobian (from linearize)
%
%  Outputs:
%    sys_lon - struct with fields A [5x5], B [5x2],
%              states {'u','w','q','theta','h'}, inputs {'delta_e','delta_t'}
%    sys_lat - struct with fields A [5x5], B [5x2],
%              states {'v','p','r','phi','psi'}, inputs {'delta_a','delta_r'}
%
%  Usage:
%    [sys_lon, sys_lat] = decouple_models(A, B);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 5, Slides 37-40
% =========================================================================

function [sys_lon, sys_lat] = decouple_models(A, B)

    % -----------------------------------------------------------------------
    % STATE INDEX MAP  (1-based, matching linearize.m ordering)
    %   1=pn  2=pe  3=pd  4=u  5=v  6=w  7=phi  8=theta  9=psi  10=p  11=q  12=r
    % -----------------------------------------------------------------------

    % Longitudinal: u(4), w(6), q(11), theta(8), pd(3)  → convert pd→h
    lon_state_idx = [4, 6, 11, 8, 3];
    lon_input_idx = [1, 4];            % delta_e(1), delta_t(4)

    % Lateral: v(5), p(10), r(12), phi(7), psi(9)
    lat_state_idx = [5, 10, 12, 7, 9];
    lat_input_idx = [2, 3];            % delta_a(2), delta_r(3)

    % -----------------------------------------------------------------------
    % EXTRACT RAW SUBMATRICES
    % -----------------------------------------------------------------------
    A_lon_raw = A(lon_state_idx, lon_state_idx);   % 5x5
    B_lon_raw = B(lon_state_idx, lon_input_idx);   % 5x2

    A_lat_raw = A(lat_state_idx, lat_state_idx);   % 5x5
    B_lat_raw = B(lat_state_idx, lat_input_idx);   % 5x2

    % -----------------------------------------------------------------------
    % CONVERT pd → h  FOR LONGITUDINAL MODEL
    %   h = -pd   →   h_dot = -pd_dot
    %   Transform: T = diag([1,1,1,1,-1])
    %   A_lon = T * A_raw * T   (row 5 and col 5 sign-flipped; (5,5) cancels)
    %   B_lon = T * B_raw       (row 5 sign-flipped)
    %
    %   Physical meaning: row 5 is the altitude equation (h_dot = Va*sin(θ))
    %   At trim: h_dot ≈ Va*(theta - alpha) = Va*gamma  (positive = climbing)
    % -----------------------------------------------------------------------
    T_lon = diag([1, 1, 1, 1, -1]);
    A_lon = T_lon * A_lon_raw * T_lon;
    B_lon = T_lon * B_lon_raw;

    % -----------------------------------------------------------------------
    % PACK OUTPUT STRUCTS
    % -----------------------------------------------------------------------
    sys_lon.A      = A_lon;
    sys_lon.B      = B_lon;
    sys_lon.states = {'u', 'w', 'q', 'theta', 'h'};
    sys_lon.inputs = {'delta_e', 'delta_t'};

    sys_lat.A      = A_lat_raw;   % no sign flip needed for lateral
    sys_lat.B      = B_lat_raw;
    sys_lat.states = {'v', 'p', 'r', 'phi', 'psi'};
    sys_lat.inputs = {'delta_a', 'delta_r'};

end
