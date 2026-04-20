% =========================================================================
%  CH10_V2 - Chapter 10 Adım Adım Test
%  Her section'ı ayrı ayrı çalıştır (Ctrl+Enter)
% =========================================================================

%% Section 1: Straight Path Following - Basit Senaryo
% UAV (0,0)'da başlıyor, (400,0) hedef — düz kuzey yönü
clc;
fprintf('=== Section 1: Straight Path Following ===\n\n');

% Path tanımla: WP1(0,0) -> WP2(400,0), kuzey yönü
path.type = 'line';
path.r    = [0; 0; -100];        % başlangıç noktası [pn,pe,pd]
path.q    = [1; 0; 0];           % kuzey yönü (qn=1, qe=0)
path.Va   = 25;

% Farklı pozisyonlarda test et
test_cases = [
    0,   0,  -100,  0;       % tam path üzerinde, kuzey bakıyor
    0,  50,  -100,  0;       % 50m sağda (doğu), kuzey bakıyor
    0, -50,  -100,  0;       % 50m solda (batı), kuzey bakıyor
    100, 30, -100,  pi/6;    % yolda 100m ilerde, 30m sağda
];

fprintf('%-6s %-8s %-8s %-10s %-10s\n', 'Test', 'pn', 'pe', 'chi_c(deg)', 'e_py(m)');
fprintf('%s\n', repmat('-',1,48));

for i = 1:size(test_cases,1)
    pos = test_cases(i, 1:3)';
    chi = test_cases(i, 4);
    [chi_c, h_c, e_py] = follow_straight_line(path, pos, chi);
    fprintf('%-6d %-8.0f %-8.0f %-10.1f %-10.1f\n', ...
        i, pos(1), pos(2), rad2deg(chi_c), e_py);
end

fprintf('\nBeklenen:\n');
fprintf('  Test 1: chi_c = 0 deg  (tam yolda, duz kuzey)\n');
fprintf('  Test 2: chi_c < 0 deg  (saga cekis, sola don)\n');
fprintf('  Test 3: chi_c > 0 deg  (sola cekis, saga don)\n');
fprintf('  Test 4: kucuk pozitif aci (hafif sag sapma)\n');
