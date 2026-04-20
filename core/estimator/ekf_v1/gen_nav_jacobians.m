% =========================================================================
%  GEN_NAV_JACOBIANS - Symbolic Jacobian generator for AtlasFC 13-state EKF
% =========================================================================
%  Run ONCE (requires Symbolic Math Toolbox) to generate:
%    nav_jac_A.m  — ∂f/∂x  (13×13 state Jacobian)
%    nav_jac_B.m  — ∂f/∂u  (13×6  input Jacobian)
%
%  These generated files are called by nav_equations.m at runtime.
%  nav_equations.m itself does NOT need to change when states are added.
%
%  ── HOW TO ADD A NEW STATE ────────────────────────────────────────────
%  1. Add a new syms variable (e.g.  syms ba_x ba_y ba_z real)
%  2. Add it to x_sym vector
%  3. Add the new dynamics equation to f_nav
%  4. Re-run this script  →  nav_jac_A.m and nav_jac_B.m regenerate
%  5. Update ekf_params.m (Q, P0 rows/cols) and ekf_init.m
%  ─────────────────────────────────────────────────────────────────────
%
%  STATE VECTOR (13×1):
%    x = [pn, pe, pd, u, v, w, e0, e1, e2, e3, bp, bq, br]
%
%  INPUT VECTOR (6×1):
%    u_imu = [y_gx, y_gy, y_gz, y_ax, y_ay, y_az]
%
%  Author : AtlasFC  |  Ref: Beard & McLain Ch. 8
% =========================================================================

atlas_root = fullfile(pwd, '..', '..');
addpath(atlas_root);
setup_paths(atlas_root);

fprintf('=========================================\n');
fprintf('  GEN_NAV_JACOBIANS  (Symbolic Toolbox)\n');
fprintf('=========================================\n\n');

% -------------------------------------------------------------------------
%  Symbolic variables
% -------------------------------------------------------------------------
syms pn pe pd u_b v_b w_b e0 e1 e2 e3 bp bq br real   % 13 states
syms y_gx y_gy y_gz y_ax y_ay y_az                    real   % 6 IMU inputs
syms g_                                                real   % gravity

% -------------------------------------------------------------------------
%  Bias-corrected angular rate
% -------------------------------------------------------------------------
pc = y_gx - bp;
qc = y_gy - bq;
rc = y_gz - br;

% -------------------------------------------------------------------------
%  Position kinematics:  ṗ = R_vb * v_body
%
%  R_vb (NED←body) = R_bv'  where R_bv from quaternion:
%    R_bv = [ e0²+e1²-e2²-e3²,  2(e1e2+e0e3),  2(e1e3-e0e2) ;
%             2(e1e2-e0e3),  e0²-e1²+e2²-e3²,  2(e2e3+e0e1) ;
%             2(e1e3+e0e2),  2(e2e3-e0e1),  e0²-e1²-e2²+e3² ]
%  R_vb = R_bv'
% -------------------------------------------------------------------------
pn_dot = (e0^2+e1^2-e2^2-e3^2)*u_b  +  2*(e1*e2-e0*e3)*v_b  +  2*(e1*e3+e0*e2)*w_b;
pe_dot = 2*(e1*e2+e0*e3)*u_b  +  (e0^2-e1^2+e2^2-e3^2)*v_b  +  2*(e2*e3-e0*e1)*w_b;
pd_dot = 2*(e1*e3-e0*e2)*u_b  +  2*(e2*e3+e0*e1)*v_b  +  (e0^2-e1^2-e2^2+e3^2)*w_b;

% -------------------------------------------------------------------------
%  Velocity:  v̇ = -ω_c × v + y_accel + g_body
%
%  -cross(ω_c, vb) = [rc*v - qc*w; pc*w - rc*u; qc*u - pc*v]
%  g_body = R_bv*[0;0;g] = g*[2(e1e3-e0e2); 2(e2e3+e0e1); e0²-e1²-e2²+e3²]
% -------------------------------------------------------------------------
u_dot = rc*v_b - qc*w_b  +  y_ax  +  2*g_*(e1*e3 - e0*e2);
v_dot = pc*w_b - rc*u_b  +  y_ay  +  2*g_*(e2*e3 + e0*e1);
w_dot = qc*u_b - pc*v_b  +  y_az  +  g_*(e0^2 - e1^2 - e2^2 + e3^2);

% -------------------------------------------------------------------------
%  Quaternion kinematics:  q̇ = 0.5 * Ξ(q) * ω_c
%
%  Ξ(q) = [-e1, -e2, -e3 ;
%            e0, -e3,  e2 ;
%            e3,  e0, -e1 ;
%           -e2,  e1,  e0 ]
% -------------------------------------------------------------------------
e0_dot = sym(1)/2 * (-e1*pc - e2*qc - e3*rc);
e1_dot = sym(1)/2 * ( e0*pc - e3*qc + e2*rc);
e2_dot = sym(1)/2 * ( e3*pc + e0*qc - e1*rc);
e3_dot = sym(1)/2 * (-e2*pc + e1*qc + e0*rc);

% -------------------------------------------------------------------------
%  Bias:  ḃ = 0  (random walk — driven by process noise Q)
% -------------------------------------------------------------------------
bp_dot = sym(0);
bq_dot = sym(0);
br_dot = sym(0);

% -------------------------------------------------------------------------
%  Navigation equations  f(x, u_imu)  →  13×1
%  (add new rows here when adding states)
% -------------------------------------------------------------------------
f_nav = [pn_dot; pe_dot; pd_dot; ...
         u_dot;  v_dot;  w_dot;  ...
         e0_dot; e1_dot; e2_dot; e3_dot; ...
         bp_dot; bq_dot; br_dot];

% -------------------------------------------------------------------------
%  State and input symbol vectors  (column vectors → generated code uses
%  in1(k,1) indexing which accepts 13×1 column vector inputs at runtime)
%  (add new symbols here when adding states/inputs)
% -------------------------------------------------------------------------
x_sym = [pn; pe; pd; u_b; v_b; w_b; e0; e1; e2; e3; bp; bq; br];  % 13×1
u_sym = [y_gx; y_gy; y_gz; y_ax; y_ay; y_az];                      % 6×1

% -------------------------------------------------------------------------
%  Compute Jacobians
% -------------------------------------------------------------------------
fprintf('Computing A = ∂f/∂x  (13×13) ...\n');
A_sym = jacobian(f_nav, x_sym);

fprintf('Computing B = ∂f/∂u  (13×6)  ...\n');
B_sym = jacobian(f_nav, u_sym);

fprintf('Jacobians computed.\n\n');

% -------------------------------------------------------------------------
%  Generate MATLAB function files
%  Output directory: same folder as nav_equations.m
% -------------------------------------------------------------------------
out_dir = fileparts(which('nav_equations'));
if isempty(out_dir)
    out_dir = pwd;
    warning('nav_equations not found on path. Saving to current directory.');
end

fprintf('Generating nav_jac_A.m ...\n');
matlabFunction(A_sym, ...
    'File',    fullfile(out_dir, 'nav_jac_A'), ...
    'Vars',    {x_sym, u_sym, g_}, ...
    'Outputs', {'A'});

fprintf('Generating nav_jac_B.m ...\n');
matlabFunction(B_sym, ...
    'File',    fullfile(out_dir, 'nav_jac_B'), ...
    'Vars',    {x_sym, u_sym, g_}, ...
    'Outputs', {'B'});

fprintf('\n=========================================\n');
fprintf('  DONE.\n');
fprintf('  nav_jac_A.m  →  %s\n', out_dir);
fprintf('  nav_jac_B.m  →  %s\n', out_dir);
fprintf('=========================================\n\n');
fprintf('These files are called by nav_equations.m.\n');
fprintf('Re-run this script whenever states change.\n\n');
