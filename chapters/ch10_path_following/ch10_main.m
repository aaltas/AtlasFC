% =========================================================================
%  CH10_MAIN - Chapter 10: Dubins Path Manager
% =========================================================================
%  Demonstrates Dubins-path following using arc-line-arc segments.
%  Compares trajectory against the Ch9 straight-line result to show
%  the effect of minimum-turning-radius constraints.
%
%  Run from the AtlasFC project root after setup_paths.
%
%  OUTPUTS:
%    • 5 plot windows (2-D trajectory, altitude, course, Dubins details,
%      Ch9 vs Ch10 overlay)
%    • Console: Dubins path statistics and tracking metrics
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 10
% =========================================================================

clc;
fprintf('\n========================================\n');
fprintf('  AtlasFC  --  Chapter 10: Dubins Path Manager\n');
fprintf('========================================\n\n');

% -----------------------------------------------------------------------
%  Section 1: Setup
% -----------------------------------------------------------------------
cfg10 = cfg_ch10();
cfg09 = cfg_ch09();   % Ch9 for comparison

fprintf('--- Dubins Parameters ---\n');
fprintf('  R_min    = %.0f m\n', cfg10.R_min);
fprintf('  Sim time = %.0f s  (Ch9: %.0f s)\n', cfg10.T, cfg09.T);
fprintf('  Waypoints: [%d x 4] matrix\n', size(cfg10.waypoints,1));
fprintf('    Rectangle %d×%d m at %d m altitude\n\n', ...
    max(cfg10.waypoints(:,1)), max(cfg10.waypoints(:,2)), ...
    round(-cfg10.waypoints(1,3)));

% -----------------------------------------------------------------------
%  Section 2: Run Dubins simulation (Ch10)
% -----------------------------------------------------------------------
fprintf('--- SECTION 2: Dubins Simulation (%g s) ---\n', cfg10.T);
fprintf('  Simulation time: %.1f s (%d Hz, %d steps)\n', ...
    cfg10.T, round(1/cfg10.dt), round(cfg10.T/cfg10.dt));
fprintf('  Running simulation...\n');
t0 = tic;
r10 = atlas_sim(cfg10);
elapsed = toc(t0);
fprintf('  Done in %.2f s (%d steps).\n\n', elapsed, length(r10.t));

% -----------------------------------------------------------------------
%  Section 3: Run Ch9 straight-line simulation for comparison
% -----------------------------------------------------------------------
fprintf('--- SECTION 3: Ch9 Straight-Line Simulation (%.0f s) ---\n', cfg09.T);
fprintf('  Running...\n');
r09 = atlas_sim(cfg09);
fprintf('  Done.\n\n');

% -----------------------------------------------------------------------
%  Section 4: Tracking performance metrics
%
%  NOTE on metrics for Dubins path following:
%    chi_cmd is NOT stored per-step for orbit segments (the guidance law
%    computes chi_c from orbit geometry each step; only the step-reference
%    is saved in results.cmd.chi). Meaningful metrics are therefore:
%      • Altitude deviation from nominal (100 m)
%      • Airspeed error (always available)
%      • Cross-track deviation from waypoint lines (computed below)
% -----------------------------------------------------------------------
fprintf('--- SECTION 4: Tracking Performance ---\n');
dt = cfg10.dt;
N10 = length(r10.t);

% Altitude — compare against nominal h0 = 100 m (all waypoints at 100 m)
h_nom  = cfg10.h0 * ones(1, N10);
h_err  = r10.true.h - h_nom;
Va_err = r10.true.Va - r10.cmd.Va;

% Cross-track error — lateral distance from nearest waypoint line segment
xte = cross_track_error(r10.true.pn, r10.true.pe, cfg10.waypoints);

% Use second half of sim (allow Dubins convergence in first half)
idx_ss = max(1, round(N10/2)) : N10;

fprintf('  Altitude deviation from 100 m:\n');
fprintf('    RMS: %.2f m   Max: %.2f m\n', rms(h_err(idx_ss)), max(abs(h_err(idx_ss))));
fprintf('    (bank-induced; phi~%.1f deg at R_min=%.0fm, Va=%.0fm/s)\n', ...
    rad2deg(atan(cfg10.Va_init^2/(cfg10.R_min*9.81))), cfg10.R_min, cfg10.Va_init);
fprintf('  Cross-track to waypoint LINES (informational only):\n');
fprintf('    RMS: %.1f m   Max: %.1f m\n', rms(xte(idx_ss)), max(abs(xte(idx_ss))));
fprintf('    NOTE: Dubins arcs intentionally deviate ~R_min from straight lines.\n');
fprintf('          Expected max deviation ≈ %.0f m for R_min=%.0f m.\n', ...
    cfg10.R_min*1.5, cfg10.R_min);
fprintf('    Visual check: Figure 1 should show ROUNDED corners vs Ch9 sharp corners.\n');
fprintf('  Airspeed:\n');
fprintf('    RMS: %.4f m/s  Max: %.4f m/s\n\n', ...
    rms(Va_err(idx_ss)), max(abs(Va_err(idx_ss))));

% -----------------------------------------------------------------------
%  Section 5: Plots
% -----------------------------------------------------------------------
fprintf('  Plotting Dubins path-following results...\n');

t10 = r10.t;
t09 = r09.t;
WP  = cfg10.waypoints;

% -- Figure 1: 2-D trajectory comparison (Ch9 vs Ch10) --
figure(1); clf;
plot(r09.true.pe, r09.true.pn, 'b--', 'LineWidth', 1.2, 'DisplayName', 'Ch9 straight-line');
hold on;
plot(r10.true.pe, r10.true.pn, 'r-',  'LineWidth', 1.5, 'DisplayName', 'Ch10 Dubins');
plot(WP(:,2), WP(:,1), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'DisplayName', 'Waypoints');
for i = 1:size(WP,1)
    text(WP(i,2)+8, WP(i,1)+8, sprintf('WP%d',i), 'FontSize', 9);
end
xlabel('East [m]'); ylabel('North [m]');
title('Ch9 vs Ch10: 2-D Ground Track');
legend('Location','best'); grid on; axis equal;

% -- Figure 2: Altitude history --
figure(2); clf;
plot(t10, r10.true.h, 'r-', 'LineWidth', 1.5);
hold on;
plot(t10, r10.cmd.h,  'k--', 'LineWidth', 1, 'DisplayName', 'Commanded');
xlabel('Time [s]'); ylabel('Altitude [m]');
title('Ch10: Altitude Tracking'); legend({'True','Commanded'}); grid on;

% -- Figure 3: Course angle history --
figure(3); clf;
plot(t10, rad2deg(r10.true.chi), 'r-', 'LineWidth', 1.5);
hold on;
plot(t10, rad2deg(r10.cmd.chi),  'k--', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Course [deg]');
title('Ch10: Course Tracking'); legend({'True','Commanded'}); grid on;

% -- Figure 4: Airspeed history --
figure(4); clf;
plot(t10, r10.true.Va, 'r-', 'LineWidth', 1.5);
hold on;
plot(t10, r10.cmd.Va,  'k--', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Airspeed [m/s]');
title('Ch10: Airspeed Tracking'); legend({'True','Commanded'}); grid on;

% -- Figure 5: Ch9 vs Ch10 path length and corner geometry --
figure(5); clf;
subplot(2,1,1);
bar([1,2], [path_length(r09.true), path_length(r10.true)]);
set(gca,'XTickLabel',{'Ch9 (straight)','Ch10 (Dubins)'});
ylabel('Path length [m]'); title('Total Path Length Comparison'); grid on;

subplot(2,1,2);
text(0.1, 0.6, sprintf('Ch9 path length:  %.0f m', path_length(r09.true)), ...
    'Units','normalized', 'FontSize', 11);
text(0.1, 0.4, sprintf('Ch10 path length: %.0f m', path_length(r10.true)), ...
    'Units','normalized', 'FontSize', 11);
text(0.1, 0.2, sprintf('R_{min} = %.0f m  (Dubins arcs respect turning radius)', cfg10.R_min), ...
    'Units','normalized', 'FontSize', 10, 'Color', [0.5 0 0]);
axis off;

fprintf('\n========================================\n');
fprintf('  Chapter 10 complete. 5 plot windows.\n');
fprintf('  Dubins arc-line-arc path management.\n');
fprintf('========================================\n\n');

% =========================================================================
%  Local helpers
% =========================================================================

function L = path_length(traj)
    dpn = diff(traj.pn);
    dpe = diff(traj.pe);
    L   = sum(sqrt(dpn.^2 + dpe.^2));
end

function xte = cross_track_error(pn, pe, waypoints)
    % Compute lateral (cross-track) distance from nearest waypoint line segment.
    N = length(pn);
    xte = zeros(1, N);
    for k = 1:N
        pos = [pn(k); pe(k)];
        best = inf;
        for seg = 1 : size(waypoints,1)-1
            w1 = waypoints(seg,   1:2)';
            w2 = waypoints(seg+1, 1:2)';
            v  = w2 - w1;
            lv = norm(v);
            if lv < 1e-6; continue; end
            u  = v / lv;
            t  = dot(pos - w1, u);
            t  = max(0, min(lv, t));      % clamp to segment
            closest = w1 + t * u;
            d = norm(pos - closest);
            if d < best; best = d; end
        end
        xte(k) = best;
    end
end

function y = rms(x)
    y = sqrt(mean(x.^2));
end
