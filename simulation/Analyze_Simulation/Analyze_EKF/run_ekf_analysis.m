% =========================================================================
%  RUN_EKF_ANALYSIS  —  Monte Carlo EKF Consistency Analysis (NEES + NIS)
% =========================================================================
%  Runs the Ch8 full pipeline N_MC times with independent noise seeds,
%  accumulates NEES (Normalized Estimation Error Squared) and NIS
%  (Normalized Innovation Squared), then checks filter consistency
%  against chi-squared 95% confidence bounds.
%
%  WHAT THIS TELLS YOU:
%    • NEES > expected  →  P too small (filter overconfident)
%    • NEES < expected  →  P too large (filter underconfident / conservative)
%    • NEES ≈ expected  →  filter is consistent (covariance is accurate)
%    • Same logic applies to NIS for each sensor
%
%  EXPECTED VALUES (averaged over N_MC runs):
%    NEES_pos : 3   (3 position states)
%    NEES_vel : 3   (3 velocity states)
%    NIS_baro : 1   (scalar measurement)
%    NIS_pitot: 1
%    NIS_mag  : 1
%    NIS_gps  : 4   (4-component GPS measurement)
%
%  Chi-squared 95% bounds for sample mean of N_MC runs:
%    [chi2inv(0.025, N_MC*df) / N_MC,  chi2inv(0.975, N_MC*df) / N_MC]
%
%  Usage  (from MATLAB, any directory):
%    Run this file directly (F5 or "Run" button)
%
%  Author : AtlasFC  |  Ref: Bar-Shalom et al., "Estimation with
%           Applications to Tracking and Navigation", Ch. 5
% =========================================================================

% -------------------------------------------------------------------------
%  0. Bootstrap — find atlas root and setup paths
% -------------------------------------------------------------------------
candidate = pwd;
atlas_root = '';
for i = 1:10
    if exist(fullfile(candidate, 'setup_paths.m'), 'file')
        atlas_root = candidate;
        break;
    end
    parent = fileparts(candidate);
    if strcmp(parent, candidate), break; end
    candidate = parent;
end
if isempty(atlas_root)
    error('Could not find atlas root (setup_paths.m). Run from inside the AtlasFC project.');
end
addpath(atlas_root);
setup_paths(atlas_root);

clc;
fprintf('=========================================\n');
fprintf('  EKF Consistency Analysis  (NEES + NIS)\n');
fprintf('=========================================\n\n');

% -------------------------------------------------------------------------
%  1. Load analysis config  →  sim config  →  merge
% -------------------------------------------------------------------------
acfg = analyze_cfg();                      % analysis-level settings
cfg  = feval(acfg.sim_config);             % base sim config (e.g. cfg_ch08)
cfg.ekf_version  = acfg.ekf_version;      % EKF version from analyze_cfg
cfg.ekf_analysis = acfg.ekf_analysis;     % always true here
N_MC             = acfg.N_MC;

N  = round(cfg.T / cfg.dt);

fprintf('Sim config   : %s\n',   acfg.sim_config);
fprintf('EKF version  : %s\n',   acfg.ekf_version);
fprintf('MC runs      : %d\n',   N_MC);
fprintf('Duration     : %.0f s  (dt = %.3f s  →  %d steps/run)\n', ...
        cfg.T, cfg.dt, N);

% -------------------------------------------------------------------------
%  1b. Warmup run — measure actual per-run time and project total
% -------------------------------------------------------------------------
fprintf('\nMeasuring single-run time...');
cfg_warm          = cfg;
cfg_warm.rng_seed = 9999;   % fixed warmup seed — not part of MC sequence
t_w               = tic;
atlas_sim(cfg_warm);
t_per_run         = toc(t_w);
t_est             = t_per_run * N_MC;

fprintf('\r  Single run : %.2f s\n', t_per_run);
if t_est < 60
    fprintf('  Estimated  : %.0f s total\n', t_est);
else
    fprintf('  Estimated  : %.0f s  (%.1f min) total\n', t_est, t_est/60);
end

% -------------------------------------------------------------------------
%  1c. Confirmation prompt  (Enter = start,  Esc = cancel)
% -------------------------------------------------------------------------
fprintf('\n');
key_pressed = confirm_prompt('EKF Analysis — Ready to start?', ...
    sprintf('ENTER  to run %d Monte Carlo sims   |   ESC  to cancel', N_MC));

if ismember(key_pressed, {'escape','timeout'})
    if strcmp(key_pressed, 'timeout')
        fprintf('  No key pressed within 60 s — cancelled.\n');
    else
        fprintf('  Cancelled.\n');
    end
    return;
end
fprintf('  Starting...\n\n');

fprintf('Running Monte Carlo...\n');

% -------------------------------------------------------------------------
%  2. Pre-allocate accumulators
% -------------------------------------------------------------------------

nees_pos_all  = zeros(N_MC, N);
nees_vel_all  = zeros(N_MC, N);
nis_baro_all  = NaN(N_MC, N);
nis_pitot_all = NaN(N_MC, N);
nis_mag_all   = NaN(N_MC, N);
nis_gps_all   = NaN(N_MC, N);

t_vec = [];

% -------------------------------------------------------------------------
%  3. Monte Carlo loop
% -------------------------------------------------------------------------
BAR_W   = 36;          % # characters for the filled portion of the bar
t_start = tic;

for mc = 1:N_MC
    cfg.rng_seed = acfg.mc_seed_base + mc - 1;
    res          = atlas_sim(cfg);

    if isempty(t_vec)
        t_vec = res.t;
    end

    nees_pos_all(mc,:)  = res.nees.pos;
    nees_vel_all(mc,:)  = res.nees.vel;
    nis_baro_all(mc,:)  = res.nis.baro;
    nis_pitot_all(mc,:) = res.nis.pitot;
    nis_mag_all(mc,:)   = res.nis.mag;
    nis_gps_all(mc,:)   = res.nis.gps;

    % --- Progress bar (overwrites same line with \r) ---
    elapsed  = toc(t_start);
    eta      = elapsed / mc * (N_MC - mc);
    pct      = mc / N_MC;
    filled   = round(pct * BAR_W);
    bar_str  = [repmat('#', 1, filled), repmat('-', 1, BAR_W - filled)];

    if mc < N_MC
        fprintf('\r  [%s] %3.0f%%  run %d/%d  elapsed %4.0fs  ETA %4.0fs  ', ...
                bar_str, pct*100, mc, N_MC, elapsed, eta);
    else
        fprintf('\r  [%s] 100%%  %d/%d runs  total %.1f s               \n', ...
                bar_str, N_MC, N_MC, elapsed);
    end
end

fprintf('Done.\n\n');

% -------------------------------------------------------------------------
%  4. Compute mean NEES / NIS  (average across MC runs at each time step)
% -------------------------------------------------------------------------
mean_nees_pos  = mean(nees_pos_all,  1);
mean_nees_vel  = mean(nees_vel_all,  1);
mean_nis_baro  = nanmean(nis_baro_all,  1);
mean_nis_pitot = nanmean(nis_pitot_all, 1);
mean_nis_mag   = nanmean(nis_mag_all,   1);
mean_nis_gps   = nanmean(nis_gps_all,   1);

% -------------------------------------------------------------------------
%  5. Chi-squared 95% confidence bounds
%
%     If the filter is consistent, the sample mean of N_MC NEES values
%     at each time step follows:  (N_MC * mean_NEES) ~ chi2(N_MC * df)
%     So the bounds on mean_NEES are chi2inv([0.025,0.975], N_MC*df) / N_MC
% -------------------------------------------------------------------------
df_pos  = 3;   % position states
df_vel  = 3;   % velocity states
df_baro = 1;
df_pitot= 1;
df_mag  = 1;
df_gps  = 4;

% chi2inv requires Statistics & Machine Learning Toolbox.
% Fallback: use Wilson-Hilferty normal approximation if toolbox absent.
if license('test','statistics_toolbox') && ~isempty(which('chi2inv'))
    bnd = @(df) chi2inv([0.025, 0.975], N_MC * df) / N_MC;
else
    warning('Statistics Toolbox not found. Using chi-squared normal approximation.');
    bnd = @(df) chi2inv_approx([0.025, 0.975], N_MC * df) / N_MC;
end

bnd_pos  = bnd(df_pos);
bnd_vel  = bnd(df_vel);
bnd_baro = bnd(df_baro);
bnd_pitot= bnd(df_pitot);
bnd_mag  = bnd(df_mag);
bnd_gps  = bnd(df_gps);

% -------------------------------------------------------------------------
%  6. Consistency report
% -------------------------------------------------------------------------
check = @(label, mn, bounds, expected) ...
    fprintf('  %-14s  mean=%.2f  bounds=[%.2f, %.2f]  expected=%.0f  %s\n', ...
        label, nanmean(mn), bounds(1), bounds(2), expected, ...
        consistency_verdict(nanmean(mn), bounds, expected));

fprintf('── Filter Consistency Report ────────────────────────────────\n');
check('NEES pos',   mean_nees_pos,  bnd_pos,   df_pos);
check('NEES vel',   mean_nees_vel,  bnd_vel,   df_vel);
check('NIS baro',   mean_nis_baro,  bnd_baro,  df_baro);
check('NIS pitot',  mean_nis_pitot, bnd_pitot, df_pitot);
check('NIS mag',    mean_nis_mag,   bnd_mag,   df_mag);
check('NIS GPS',    mean_nis_gps,   bnd_gps,   df_gps);
fprintf('─────────────────────────────────────────────────────────────\n\n');

% Fraction of time inside bounds
frac_in = @(mn, lo, hi) mean(mn >= lo & mn <= hi) * 100;
fprintf('Fraction of time steps inside 95%% bounds:\n');
fprintf('  NEES pos   : %5.1f%%\n', frac_in(mean_nees_pos,  bnd_pos(1),   bnd_pos(2)));
fprintf('  NEES vel   : %5.1f%%\n', frac_in(mean_nees_vel,  bnd_vel(1),   bnd_vel(2)));
fprintf('  NIS baro   : %5.1f%%\n', frac_in(mean_nis_baro,  bnd_baro(1),  bnd_baro(2)));
fprintf('  NIS pitot  : %5.1f%%\n', frac_in(mean_nis_pitot, bnd_pitot(1), bnd_pitot(2)));
fprintf('  NIS mag    : %5.1f%%\n', frac_in(mean_nis_mag,   bnd_mag(1),   bnd_mag(2)));
fprintf('  NIS GPS    : %5.1f%%\n', frac_in(mean_nis_gps,   bnd_gps(1),   bnd_gps(2)));
fprintf('\n');

% -------------------------------------------------------------------------
%  7. Plots
% -------------------------------------------------------------------------
figure_handles = [];

% --- Figure 1: NEES ---
fh = figure('Name','EKF NEES — Position & Velocity','NumberTitle','off');
figure_handles(end+1) = fh;

subplot(2,1,1);
plot_nees_panel(t_vec, mean_nees_pos, bnd_pos, df_pos, ...
    'NEES — Position  (expected = 3)', N_MC);

subplot(2,1,2);
plot_nees_panel(t_vec, mean_nees_vel, bnd_vel, df_vel, ...
    'NEES — Velocity  (expected = 3)', N_MC);

% --- Figure 2: NIS (scalar sensors) ---
fh = figure('Name','EKF NIS — Scalar Sensors','NumberTitle','off');
figure_handles(end+1) = fh;

subplot(3,1,1);
plot_nees_panel(t_vec, mean_nis_baro,  bnd_baro,  df_baro,  ...
    'NIS — Barometer  (expected = 1)', N_MC);

subplot(3,1,2);
plot_nees_panel(t_vec, mean_nis_pitot, bnd_pitot, df_pitot, ...
    'NIS — Pitot  (expected = 1)', N_MC);

subplot(3,1,3);
plot_nees_panel(t_vec, mean_nis_mag,   bnd_mag,   df_mag,   ...
    'NIS — Magnetometer  (expected = 1)', N_MC);

% --- Figure 3: NIS (GPS) ---
fh = figure('Name','EKF NIS — GPS','NumberTitle','off');
figure_handles(end+1) = fh;

plot_nees_panel(t_vec, mean_nis_gps, bnd_gps, df_gps, ...
    sprintf('NIS — GPS  (expected = 4,  %d MC runs)', N_MC), N_MC);
xlabel('Time [s]');

% --- Figure 4: NEES distributions (histograms) ---
fh = figure('Name','NEES Distribution (time-averaged)','NumberTitle','off');
figure_handles(end+1) = fh;

subplot(1,2,1);
plot_hist_panel(mean(nees_pos_all, 2), df_pos, bnd_pos, ...
    sprintf('Mean NEES pos  (%d runs)', N_MC));

subplot(1,2,2);
plot_hist_panel(mean(nees_vel_all, 2), df_vel, bnd_vel, ...
    sprintf('Mean NEES vel  (%d runs)', N_MC));

fprintf('Plots generated.\n');

% =========================================================================
%  LOCAL HELPER FUNCTIONS
% =========================================================================

% -------------------------------------------------------------------------
function plot_nees_panel(t, mn, bnd, expected, ttl, N_MC)
% Plot mean NEES/NIS vs time with 95% chi-squared bands and expected value
    fill([t, fliplr(t)], [bnd(1)*ones(1,length(t)), bnd(2)*ones(1,length(t))], ...
         [0.85 0.95 0.85], 'EdgeColor','none', 'FaceAlpha',0.6); hold on;
    plot(t, mn, 'b-', 'LineWidth',1.5);
    yline(expected,   'k--', 'LineWidth',1.5, 'Label',sprintf('Expected = %d', expected));
    yline(bnd(1), 'g:', 'LineWidth',1);
    yline(bnd(2), 'g:', 'LineWidth',1);
    hold off;
    xlabel('Time [s]');
    ylabel('Mean NEES/NIS');
    title(ttl);
    legend(sprintf('95%% bounds (N=%d)', N_MC), 'Mean', 'Expected', ...
           'Location','northeast');
    grid on;
end

% -------------------------------------------------------------------------
function plot_hist_panel(per_run_mean, df, bnd, ttl)
% Histogram of per-run time-averaged NEES with chi-squared reference
    histogram(per_run_mean, 'Normalization','probability', 'FaceColor',[0.4 0.6 0.8]);
    hold on;
    xline(df,      'k--', 'LineWidth',2, 'Label','Expected');
    xline(bnd(1),  'g-',  'LineWidth',1.5, 'Label','95% lo');
    xline(bnd(2),  'g-',  'LineWidth',1.5, 'Label','95% hi');
    hold off;
    xlabel('Per-run mean NEES');
    ylabel('Fraction');
    title(ttl);
    grid on;
end

% -------------------------------------------------------------------------
function v = consistency_verdict(val, bounds, expected)
% Return a short verdict string based on where val falls
    if val < bounds(1)
        v = '?  OVERESTIMATED P  (too conservative)';
    elseif val > bounds(2)
        v = '?  UNDERESTIMATED P (overconfident)';
    else
        fraction = (val - expected) / expected * 100;
        v = sprintf('OK consistent  (%.1f%% from expected)', fraction);
    end
end

% -------------------------------------------------------------------------
function key = confirm_prompt(title_str, msg_str)
% Open a small confirmation window. Returns 'return' (Enter), 'escape', or
% 'timeout' (no key pressed within 60 s). Window closes on any of those.
    hf = figure( ...
        'Name',        title_str, ...
        'NumberTitle', 'off', ...
        'MenuBar',     'none', ...
        'ToolBar',     'none', ...
        'Resize',      'off', ...
        'Position',    [500 450 420 70]);
    uicontrol(hf, ...
        'Style',               'text', ...
        'Units',               'normalized', ...
        'Position',            [0.02 0.15 0.96 0.70], ...
        'String',              msg_str, ...
        'FontSize',            11, ...
        'HorizontalAlignment', 'center', ...
        'BackgroundColor',     [0.94 0.94 0.94]);
    setappdata(hf, 'key', 'timeout');
    set(hf, 'KeyPressFcn', @(src, evt) cb_key(src, evt));
    figure(hf);          % bring to front so keypress lands here
    uiwait(hf, 60);      % auto-cancel after 60 s
    if isvalid(hf)
        key = getappdata(hf, 'key');
        close(hf);
    else
        key = 'timeout';
    end
end

% -------------------------------------------------------------------------
function cb_key(src, evt)
% KeyPressFcn callback: store key name and resume uiwait
    accepted = {'return','escape'};
    if ismember(evt.Key, accepted)
        setappdata(src, 'key', evt.Key);
        uiresume(src);
    end
    % ignore all other keys (arrows, letters, etc.)
end

% -------------------------------------------------------------------------
function x = chi2inv_approx(p, nu)
% Wilson-Hilferty chi-squared quantile approximation (no toolbox required)
%   Accuracy: error < 0.01 for nu >= 2 and p in [0.01, 0.99]
    x = zeros(size(p));
    for i = 1:numel(p)
        z = norminv_approx(p(i));   % standard normal quantile
        h = 1 - 2/(9*nu);
        s = sqrt(2/(9*nu));
        x(i) = nu * (h + s*z)^3;
        x(i) = max(x(i), 0);
    end
end

% -------------------------------------------------------------------------
function z = norminv_approx(p)
% Rational approximation to the standard normal inverse CDF (Beasley-Springer)
%   Max error ≈ 4.5e-4
    if p <= 0 || p >= 1
        z = sign(p - 0.5) * Inf;  return;
    end
    if p < 0.5
        t = sqrt(-2*log(p));
    else
        t = sqrt(-2*log(1-p));
    end
    c = [2.515517, 0.802853, 0.010328];
    d = [1.432788, 0.189269, 0.001308];
    z = t - (c(1) + c(2)*t + c(3)*t^2) / (1 + d(1)*t + d(2)*t^2 + d(3)*t^3);
    if p < 0.5, z = -z; end
end
