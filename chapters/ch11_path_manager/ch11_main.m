% =========================================================================
%  CH11_MAIN - Chapter 11: Path Manager
% =========================================================================
%  Implements and compares the three path managers from Beard & McLain Ch.11:
%
%    11.1  Straight-line waypoint manager  (Ch9, reference)
%    11.2  Fillet path manager             (NEW in Ch11)
%    11.3  Dubins airplane path manager    (Ch10, re-tested with bug fix)
%
%  All three fly the same 400×400 m rectangular loop at 100 m altitude.
%  Results are overlaid for direct comparison of corner geometry.
%
%  Sections:
%    Section 1 — Setup and parameter summary
%    Section 2 — Run all three simulations
%    Section 3 — Path-following performance comparison
%    Section 4 — Plots
%
%  Run from the AtlasFC project root after setup_paths.
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 11
% =========================================================================

% --- Auto-path setup ---
current_file = mfilename('fullpath');
[script_dir, ~, ~] = fileparts(current_file);
atlas_root = fullfile(script_dir, '..', '..');

addpath(atlas_root);
if ~exist('setup_paths', 'file')
    error('setup_paths.m not found. Check atlas_root path.');
end
setup_paths(atlas_root);

clc; rng(0);
fprintf('\n========================================\n');
fprintf('  AtlasFC  --  Chapter 11: Path Manager\n');
fprintf('========================================\n\n');

% =========================================================================
%  SECTION 1 — Setup
% =========================================================================
fprintf('--- SECTION 1: Setup ---\n');

cfg09 = cfg_ch09();   % Straight-line (reference)
cfg11 = cfg_ch11();   % Fillet
cfg10 = cfg_ch10();   % Dubins

fprintf('  Waypoints: 400×400 m rectangle at 100 m altitude\n');
fprintf('  Straight-line manager : k_path=0.025, chi_inf=45 deg\n');
fprintf('  Fillet  manager       : R_fillet = %.0f m\n', cfg11.R_fillet);
fprintf('  Dubins  manager       : R_min    = %.0f m\n\n', cfg10.R_min);

% =========================================================================
%  SECTION 2 — Simulations
% =========================================================================
fprintf('--- SECTION 2: Simulations ---\n');

fprintf('  [1/3] Straight-line (Ch9)  T=%.0f s ... ', cfg09.T);
t0 = tic; r09 = atlas_sim(cfg09); fprintf('done (%.1f s)\n', toc(t0));

fprintf('  [2/3] Fillet       (Ch11)  T=%.0f s ... ', cfg11.T);
t0 = tic; r11 = atlas_sim(cfg11); fprintf('done (%.1f s)\n', toc(t0));

fprintf('  [3/3] Dubins       (Ch10)  T=%.0f s ... ', cfg10.T);
t0 = tic; r10 = atlas_sim(cfg10); fprintf('done (%.1f s)\n\n', toc(t0));

% =========================================================================
%  SECTION 3 — Performance comparison
% =========================================================================
fprintf('--- SECTION 3: Performance Comparison ---\n');

WP = cfg09.waypoints;

% --- Path length ---
L09 = path_length(r09.true);
L11 = path_length(r11.true);
L10 = path_length(r10.true);
fprintf('  Path length flown:\n');
fprintf('    Straight-line : %.0f m\n', L09);
fprintf('    Fillet        : %.0f m  (%.0f m longer)\n', L11, L11-L09);
fprintf('    Dubins        : %.0f m  (%.0f m longer)\n\n', L10, L10-L09);

% --- Cross-track error (distance from nearest waypoint segment) ---
xte09 = cross_track_error(r09.true, WP);
xte11 = cross_track_error(r11.true, WP);
xte10 = cross_track_error(r10.true, WP);

% Steady-state: second half of each sim
N09 = length(r09.t); idx09 = round(N09/2):N09;
N11 = length(r11.t); idx11 = round(N11/2):N11;
N10 = length(r10.t); idx10 = round(N10/2):N10;

fprintf('  Cross-track error (RMS, 2nd half of sim):\n');
fprintf('    NOTE: All configs use loop_waypoints=true so UAV stays on\n');
fprintf('          the rectangle. These values reflect genuine tracking error.\n');
fprintf('    Straight-line : %.1f m\n',   rms_val(xte09(idx09)));
fprintf('    Fillet        : %.1f m  (fillet arcs deviate ~R_fillet*(1-1/sqrt2) at corners)\n', rms_val(xte11(idx11)));
fprintf('    Dubins        : %.1f m  (Dubins arcs deviate ~R_min at corners)\n\n', rms_val(xte10(idx10)));

% --- Altitude tracking ---
h09 = r09.true.h; h11 = r11.true.h; h10 = r10.true.h;
fprintf('  Altitude RMS error from 100 m:\n');
fprintf('    Straight-line : %.2f m\n',   rms_val(h09(idx09) - 100));
fprintf('    Fillet        : %.2f m\n',   rms_val(h11(idx11) - 100));
fprintf('    Dubins        : %.2f m\n\n', rms_val(h10(idx10) - 100));

% --- Corner geometry: max deviation from straight-line path at corners ---
fprintf('  Corner deviation (max dist from ideal lines at WP2,WP3,WP4):\n');
dev11 = corner_deviation(r11.true, WP);
dev10 = corner_deviation(r10.true, WP);
fprintf('    Fillet  : %.0f m  (expected ≈ R_fillet = %.0f m)\n',  dev11, cfg11.R_fillet);
fprintf('    Dubins  : %.0f m  (expected ≈ R_min    = %.0f m)\n\n', dev10, cfg10.R_min);

% =========================================================================
%  SECTION 4 — Plots
% =========================================================================
fprintf('--- SECTION 4: Plots ---\n');

t09 = r09.t;  t11 = r11.t;  t10 = r10.t;

% -- Figure 1: 2-D ground track comparison --
figure(1); clf;
plot(r09.true.pe, r09.true.pn, 'b-',  'LineWidth', 1.2, 'DisplayName', 'Ch9: straight-line');
hold on;
plot(r11.true.pe, r11.true.pn, 'g-',  'LineWidth', 2.0, 'DisplayName', sprintf('Ch11: fillet R=%.0fm', cfg11.R_fillet));
plot(r10.true.pe, r10.true.pn, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Ch10: Dubins R=%.0fm', cfg10.R_min));
plot(WP(:,2), WP(:,1), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'DisplayName', 'Waypoints');
for i = 1:size(WP,1)
    text(WP(i,2)+12, WP(i,1)+12, sprintf('WP%d',i), 'FontSize', 9);
end
xlabel('East [m]'); ylabel('North [m]');
title('Ch11: Path Manager Comparison — 2D Ground Track');
legend('Location','southeast'); grid on; axis equal;

% -- Figure 2: Altitude tracking --
figure(2); clf;
plot(t09, r09.true.h, 'b-',  'LineWidth', 1.2, 'DisplayName', 'Straight-line');
hold on;
plot(t11, r11.true.h, 'g-',  'LineWidth', 2.0, 'DisplayName', 'Fillet');
plot(t10, r10.true.h, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Dubins');
yline(100, 'k:', 'Nominal', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Altitude [m]');
title('Ch11: Altitude Tracking — All Three Managers');
legend('Location','best'); grid on;

% -- Figure 3: Course angle --
figure(3); clf;
plot(t09, rad2deg(r09.true.chi), 'b-',  'LineWidth', 1.2, 'DisplayName', 'Straight-line');
hold on;
plot(t11, rad2deg(r11.true.chi), 'g-',  'LineWidth', 2.0, 'DisplayName', 'Fillet');
plot(t10, rad2deg(r10.true.chi), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Dubins');
xlabel('Time [s]'); ylabel('Course [deg]');
title('Ch11: Course Angle — True Heading');
legend('Location','best'); grid on;

% -- Figure 4: Corner zoom — WP2 area --
figure(4); clf;
hold on;
plot(r09.true.pe, r09.true.pn, 'b-',  'LineWidth', 1.2, 'DisplayName', 'Straight-line');
plot(r11.true.pe, r11.true.pn, 'g-',  'LineWidth', 2.5, 'DisplayName', 'Fillet');
plot(r10.true.pe, r10.true.pn, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Dubins');
% Zoom around WP2 (400,0)
cx = WP(2,2); cy = WP(2,1); margin = 200;
xlim([cx-margin, cx+margin]); ylim([cy-margin, cy+margin]);
plot(WP(2,2), WP(2,1), 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
text(WP(2,2)+10, WP(2,1)+10, 'WP2', 'FontSize', 10, 'FontWeight','bold');
xlabel('East [m]'); ylabel('North [m]');
title('Ch11: Corner Geometry Zoom — WP2 (400,0)');
legend('Location','northwest'); grid on; axis equal;

% -- Figure 5: Path length bar comparison --
figure(5); clf;
bar([1,2,3], [L09, L11, L10], 0.5, 'FaceColor', [0.4 0.6 0.9]);
set(gca, 'XTickLabel', {'Straight-line','Fillet','Dubins'});
ylabel('Path length [m]');
title('Ch11: Total Path Length Comparison');
grid on;
text(1, L09+20, sprintf('%.0f m', L09), 'HorizontalAlignment','center', 'FontWeight','bold');
text(2, L11+20, sprintf('%.0f m', L11), 'HorizontalAlignment','center', 'FontWeight','bold');
text(3, L10+20, sprintf('%.0f m', L10), 'HorizontalAlignment','center', 'FontWeight','bold');

fprintf('\n========================================\n');
fprintf('  Chapter 11 complete. 5 plot windows.\n');
fprintf('  Three path managers compared.\n');
fprintf('========================================\n\n');

% =========================================================================
%  Local helpers
% =========================================================================

function L = path_length(traj)
    dpn = diff(traj.pn);  dpe = diff(traj.pe);
    L   = sum(sqrt(dpn.^2 + dpe.^2));
end

function xte = cross_track_error(traj, waypoints)
    N = length(traj.pn);
    xte = zeros(1, N);
    for k = 1:N
        pos  = [traj.pn(k); traj.pe(k)];
        best = inf;
        for seg = 1:size(waypoints,1)-1
            w1 = waypoints(seg,   1:2)';
            w2 = waypoints(seg+1, 1:2)';
            v  = w2 - w1;  lv = norm(v);
            if lv < 1e-6; continue; end
            u  = v / lv;
            t  = max(0, min(lv, dot(pos-w1, u)));
            d  = norm(pos - w1 - t*u);
            if d < best; best = d; end
        end
        xte(k) = best;
    end
end

function dev = corner_deviation(traj, waypoints)
    % Max distance from nearest waypoint SEGMENT at corner waypoints (2..N-1)
    N_wp = size(waypoints, 1);
    dev  = 0;
    for wi = 2:N_wp-1
        wc = waypoints(wi, 1:2)';
        % Find trajectory points near this waypoint (within 3× typical fillet)
        dists = sqrt((traj.pn - wc(1)).^2 + (traj.pe - wc(2)).^2);
        near  = dists < 300;
        if ~any(near); continue; end
        % Distance from nearest segment pair
        w1  = waypoints(wi-1, 1:2)';  w2 = wc;  w3 = waypoints(wi+1, 1:2)';
        d12 = seg_dist(traj.pn(near), traj.pe(near), w1, w2);
        d23 = seg_dist(traj.pn(near), traj.pe(near), w2, w3);
        local_max = max(min([d12; d23], [], 1));
        dev = max(dev, local_max);
    end
end

function d = seg_dist(pn, pe, w1, w2)
    v   = w2 - w1;  lv = norm(v);
    if lv < 1e-6; d = sqrt((pn-w1(1)).^2 + (pe-w1(2)).^2); return; end
    u   = v / lv;
    pos = [pn(:)'; pe(:)'];
    t   = u' * (pos - w1);
    t   = max(0, min(lv, t));
    cx  = w1(1) + t*u(1);  cy = w1(2) + t*u(2);
    d   = sqrt((pn(:)'-cx).^2 + (pe(:)'-cy).^2);
end

function y = rms_val(x)
    y = sqrt(mean(x.^2));
end
