% =========================================================================
%  RUN_SIM - AtlasFC main simulation runner
% =========================================================================
%  Run this file → select config → sim runs → plots open.
%
%  When adding new chapter:
%    1. Write cfg_chXX.m under configs/
%    2. Update CFG_SELECT in this file
%    3. Run
%
%  Author : AtlasFC
% =========================================================================

% --- Auto-path setup ---
% Search upward for AtlasFC root (contains setup_paths.m).
% Works regardless of MATLAB's current working directory or script depth.
candidate  = pwd;
atlas_root = '';
for i = 1:8
    if exist(fullfile(candidate, 'setup_paths.m'), 'file')
        atlas_root = candidate;
        break;
    end
    parent = fileparts(candidate);
    if strcmp(parent, candidate), break; end   % filesystem root — stop
    candidate = parent;
end
if isempty(atlas_root)
    error('AtlasFC root not found. setup_paths.m should be in a visible parent folder.');
end
addpath(atlas_root);
setup_paths(atlas_root);   % paths now set — survives the clear below

clc;

% =========================================================================
%  CONFIG SELECTION — change this, rest stays constant
% =========================================================================
CFG_SELECT = 'ch12';   % 'ch08' | 'ch09' | ...

switch CFG_SELECT
    case 'ch08'
        cfg = cfg_ch08();
    case 'ch09'
        cfg = cfg_ch09();
    case 'ch10'
        cfg = cfg_ch10();
    case 'ch11'
        cfg = cfg_ch11();
    case 'ch12'
        cfg = cfg_ch12();
    otherwise
        error('Unknown config: %s', CFG_SELECT);
end

fprintf('\n========================================\n');
fprintf('  AtlasFC  --  run_sim  [%s]\n', CFG_SELECT);
fprintf('========================================\n');
fprintf('  dt=%.3fs  T=%.0fs  EKF=%s  Guidance=%s\n\n', ...
    cfg.dt, cfg.T, mat2str(cfg.use_ekf), mat2str(cfg.guidance_on));

% =========================================================================
%  SIMULATION
% =========================================================================
tic;
results = atlas_sim(cfg);
elapsed = toc;
fprintf('  Sim completed: %.2f s (%d steps)\n\n', elapsed, round(cfg.T/cfg.dt));

% =========================================================================
%  ERROR ANALYSIS (last 20s)
% =========================================================================
idx = results.t >= 40;

Va_err    = results.est.Va(idx)  - results.true.Va(idx);
h_err     = results.est.h(idx)   - results.true.h(idx);
chi_err   = atan2(sin(results.est.chi(idx) - results.true.chi(idx)), ...
                  cos(results.est.chi(idx) - results.true.chi(idx)));
phi_err   = results.est.phi(idx)   - results.true.phi(idx);
theta_err = results.est.theta(idx) - results.true.theta(idx);

fprintf('--- Estimation Errors (last 20s) ---\n');
fprintf('  Va    : RMS=%.4f m/s    max=%.4f m/s\n',  rms(Va_err),  max(abs(Va_err)));
fprintf('  h     : RMS=%.4f m      max=%.4f m\n',    rms(h_err),   max(abs(h_err)));
fprintf('  chi   : RMS=%.4f deg    max=%.4f deg\n',  rad2deg(rms(chi_err)), rad2deg(max(abs(chi_err))));
fprintf('  phi   : RMS=%.4f deg\n', rad2deg(rms(phi_err)));
fprintf('  theta : RMS=%.4f deg\n', rad2deg(rms(theta_err)));
fprintf('\n  Gyro bias (end): bp=%.5f  bq=%.5f  br=%.5f rad/s\n\n', ...
    results.est.bp(end), results.est.bq(end), results.est.br(end));

% =========================================================================
%  PLOTS
% =========================================================================
t = results.t;

% --- Plot 1: Altitude ---
figure('Name','run_sim: Altitude','NumberTitle','off');
subplot(2,1,1);
plot(t, results.true.h, 'b', 'LineWidth', 1.5); hold on;
plot(t, results.est.h,  'r--', 'LineWidth', 1.2);
stairs(t, results.cmd.h, 'k:', 'LineWidth', 1.0);
fill([t, fliplr(t)], ...
     [results.est.h + 3*results.sigma.h, fliplr(results.est.h - 3*results.sigma.h)], ...
     'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
ylabel('h [m]'); title('Altitude'); legend('True','EKF','Command','3σ'); grid on;
subplot(2,1,2);
plot(t, results.est.h - results.true.h, 'k', 'LineWidth', 1.0);
ylabel('Error [m]'); xlabel('Time [s]'); grid on; title('Altitude Estimation Error');

% --- Plot 2: Velocity ---
figure('Name','run_sim: Airspeed','NumberTitle','off');
subplot(2,1,1);
plot(t, results.true.Va, 'b', 'LineWidth', 1.5); hold on;
plot(t, results.est.Va,  'r--', 'LineWidth', 1.2);
stairs(t, results.cmd.Va, 'k:', 'LineWidth', 1.0);
ylabel('Va [m/s]'); title('Airspeed'); legend('True','EKF','Command'); grid on;
subplot(2,1,2);
plot(t, results.est.Va - results.true.Va, 'k', 'LineWidth', 1.0);
ylabel('Error [m/s]'); xlabel('Time [s]'); grid on;

% --- Plot 3: Attitude ---
figure('Name','run_sim: Attitude','NumberTitle','off');
subplot(3,1,1);
plot(t, rad2deg(results.true.phi), 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(results.est.phi),  'r--', 'LineWidth', 1.2);
ylabel('\phi [deg]'); title('Roll'); legend('True','EKF'); grid on;
subplot(3,1,2);
plot(t, rad2deg(results.true.theta), 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(results.est.theta),  'r--', 'LineWidth', 1.2);
ylabel('\theta [deg]'); title('Pitch'); grid on;
subplot(3,1,3);
plot(t, rad2deg(results.true.chi), 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(results.est.chi),  'r--', 'LineWidth', 1.2);
stairs(t, rad2deg(results.cmd.chi), 'k:', 'LineWidth', 1.0);
ylabel('\chi [deg]'); title('Course'); xlabel('Time [s]'); grid on;

% --- Plot 4: Gyro Bias Convergence ---
figure('Name','run_sim: Gyro Bias','NumberTitle','off');
subplot(3,1,1);
plot(t, rad2deg(results.est.bp), 'b'); ylabel('b_p [deg/s]'); title('Gyro Bias Estimation'); grid on;
subplot(3,1,2);
plot(t, rad2deg(results.est.bq), 'r'); ylabel('b_q [deg/s]'); grid on;
subplot(3,1,3);
plot(t, rad2deg(results.est.br), 'g'); ylabel('b_r [deg/s]'); xlabel('Time [s]'); grid on;

fprintf('========================================\n');
fprintf('  %d plot windows opened.\n', 4);
fprintf('  results struct available in workspace.\n');
fprintf('========================================\n\n');
