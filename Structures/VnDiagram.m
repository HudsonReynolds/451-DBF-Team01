function [outputs, params] = VnDiagram(params)

%% Initializations

% Unit conversion
KG2LB = 2.20462;

% Aircraft-Specific Values
S = params.geometry.S_wing;
c_bar = params.geometry.c_wing;
CL_max = params.aero.CL_max;
CL_max_neg = params.aero.CL_max_neg;   % ASSUMPTION
CL_alpha = params.aero.CL_Alpha;
rho = params.env.rho;
g = params.env.g;

% Mass table is the source of truth for structural analysis (team
% decision, same convention WingLoads.m/TailLoads.m/FuselageLoads.m/
% TailDraggerTakeOff.m already use): use its own sum as the aircraft
% weight here, not the iterated params.performance.MTOM/MTOW. This
% matters beyond just this figure -- WingLoads.m etc. apply the load
% factors and speeds computed here to THEIR OWN mass-table weight, so
% deriving those speeds/factors from a different (iterated) total would
% silently mismatch the two.
massTable = readtable('SizingParams.xlsx', 'Sheet', 'MassBudget');
MTOM = sum(massTable.Mass_g)/1000; % kg
MTOW = MTOM * g; % N

% Self-consistent stall speed for THIS diagram's own rho/S/CL_max/MTOW
% (same method already used below for V_S_neg) -- guarantees the stall
% boundary curve passes through exactly n=1 at V_S and exactly n=n+ at
% V_A, so it stays flush with the limit-load-factor line no matter how
% CL_max or MTOW move during the sizing iteration. The Excel-input
% params.performance.V_S is an independently-set target used earlier
% (e.g. V_TO sizing) and can drift out of sync with this diagram's own
% values -- not used here for that reason.
V_S = sqrt(2*MTOW/(rho*S*CL_max));
V_C = params.performance.V_C;
V_max_level = params.performance.V_max_level;
W_S = MTOW / S; 

%% Computations 

% FAR 23.337: Limit Maneuvering Load Factor Candidates
W_lb = MTOM * KG2LB;
n_pos_raw = 2.1 + 24000/(W_lb + 10000);         % FAR 23.337(a), raw weight-formula value
n_pos_capped = min(max(n_pos_raw, 2.5), 3.8);   % clipped to the normal-category bracket

% FAR 23.341 Gust Load Factor
mu_g = 2*W_S / (rho*c_bar*g*CL_alpha);
K_g = 0.88*mu_g / (5.3+mu_g);

U_de_VC = 15.2; % m/s (50 ft/s), FAR 23.333, sea level
U_de_VD = 7.6;  % m/s (25 ft/s), FAR 23.333, sea level

gustN = @(U_de, V) 1 + (K_g*U_de*V*CL_alpha) / (2*W_S/rho);

n_gust_VC_raw = gustN(U_de_VC, V_C);
n_stall_VC = 0.5*rho*V_C^2*CL_max/W_S;
n_gust_VC_capped = min(n_gust_VC_raw, n_stall_VC);

% Adopted positive limit load factor
n_pos = n_pos_capped;

% Whether to adopt this higher gust value instead of n+ is a stated
% design decision (see lecture discussion Q1), not something the formula
% resolves on its own -- flagged in the console summary below rather
% than via warning() here, so it reads as a finding, not an error.
gust_exceeds_n_pos = n_gust_VC_capped > n_pos;

n_neg = -0.4*n_pos;      % FAR 23.337(b), using the adopted n+
n_ult_pos = 1.5*n_pos;   % ultimate = 1.5x limit (slide 6)
n_ult_neg = 1.5*n_neg;

% Important Speeds
V_A = V_S*sqrt(n_pos);                       
V_D = max(1.25*V_C, 1.25*V_max_level);      
V_NE = 0.9*V_D;                             

V_S_neg = sqrt(2*MTOW/(rho*S*abs(CL_max_neg)));
V_G = V_S_neg*sqrt(abs(n_neg));               

U_op = 5; % m/s
gustN_op = @(V) 1 + (K_g*U_op*V*CL_alpha) / (2*W_S/rho);
try
    V_NO = fzero(@(V) gustN_op(V) - n_pos, [0.1, 5*V_D]);
catch
    V_NO = V_D;
end

% Never-exceed check against A6 (Recheck with A9 Once Available)
NE_ok_A6 = V_NE > V_max_level;
if ~NE_ok_A6
    warning('V_NE (%.1f m/s) does NOT exceed the max level speed from A6 (%.1f m/s) -- V_D must increase.', V_NE, V_max_level);
end

% Stall Boundary Curves 
V_pos_curve = linspace(V_S, V_A, 200);
n_pos_curve = 0.5*rho*V_pos_curve.^2*S*CL_max/MTOW;

V_neg_curve = linspace(V_G, V_S_neg, 200); % V_G (most negative) down to V_S_neg (|n|=1)
n_neg_curve = -0.5*rho*V_neg_curve.^2*S*abs(CL_max_neg)/MTOW;

V_poly = [V_S, V_pos_curve, V_D,    V_D,    V_G,    V_neg_curve, V_S_neg];
n_poly = [0,   n_pos_curve, n_pos,  n_neg,  n_neg,  n_neg_curve, 0];

%% Plotting

blue  = [0.00 0.45 0.70];
orange= [0.90 0.35 0.00];
teal  = [0.00 0.60 0.50];

figure('Name','V-n Diagram','Color','w','WindowStyle','docked'); hold on; box on; grid on;

fill(V_poly, n_poly, blue, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');

plot(linspace(0, V_S, 50), 0.5*rho*linspace(0,V_S,50).^2*S*CL_max/MTOW, ':', 'Color', blue, 'HandleVisibility', 'off');
plot([V_S V_S], [0 n_pos_curve(1)], '-', 'Color', blue, 'LineWidth', 1.5, 'HandleVisibility', 'off');
plot(V_pos_curve, n_pos_curve, '-', 'Color', blue, 'LineWidth', 1.5, 'DisplayName', 'stall boundary');

% Dotted continuation of the SAME stall formula past V_A: the wing could
% aerodynamically pull more than n+ up here, but the structure can't take
% it, so this part isn't part of the envelope -- shown only so points like
% the gust-capped marker below (which land past V_A) visibly sit on it.
V_pos_ext = linspace(V_A, V_D, 100);
n_pos_ext = 0.5*rho*V_pos_ext.^2*S*CL_max/MTOW;
plot(V_pos_ext, n_pos_ext, ':', 'Color', blue, 'HandleVisibility', 'off');

plot(linspace(0, V_S_neg, 50), -0.5*rho*linspace(0,V_S_neg,50).^2*S*abs(CL_max_neg)/MTOW, ':', 'Color', blue, 'HandleVisibility', 'off');
plot([V_S_neg V_S_neg], [0 n_neg_curve(end)], '-', 'Color', blue, 'LineWidth', 1.5, 'HandleVisibility', 'off');
plot(V_neg_curve, n_neg_curve, '-', 'Color', blue, 'LineWidth', 1.5, 'HandleVisibility', 'off');

plot([V_A V_D], [n_pos n_pos], '-', 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'limit load factors');
plot([V_D V_D], [n_neg n_pos], '-', 'Color', orange, 'LineWidth', 1.5, 'HandleVisibility', 'off');
plot([V_G V_D], [n_neg n_neg], '-', 'Color', orange, 'LineWidth', 1.5, 'HandleVisibility', 'off');

V_gust = linspace(0, V_D, 200);
plot(V_gust, gustN(U_de_VC, V_gust), '--', 'Color', teal, 'DisplayName', sprintf('FAR 23 gust, %.1f m/s at $V_C$ (raw)', U_de_VC));
plot(V_gust, 2 - gustN(U_de_VC, V_gust), '--', 'Color', teal, 'HandleVisibility', 'off');
plot(V_gust, gustN(U_de_VD, V_gust), '-.', 'Color', teal, 'DisplayName', sprintf('FAR 23 gust, %.1f m/s at $V_D$ (raw)', U_de_VD));
plot(V_gust, 2 - gustN(U_de_VD, V_gust), '-.', 'Color', teal, 'HandleVisibility', 'off');

V_op_line = linspace(0, V_NO, 100);
plot(V_op_line, gustN_op(V_op_line), '-', 'Color', [0.9 0.6 0.0], 'LineWidth', 1.3, 'DisplayName', sprintf('operational gust, %.0f m/s (design)', U_op));

y_top = max(n_pos, n_gust_VC_capped) + 1.2;
y_bot = n_neg - 1.1;
ylim([y_bot, y_top]);
xlim([0, V_D*1.08]);

plot(V_C, n_gust_VC_capped, 'd', 'Color', teal, 'MarkerFaceColor', teal, 'MarkerSize', 8, 'HandleVisibility', 'off');
text(V_C, n_gust_VC_capped - 0.1, sprintf('%.1f m/s gust at $V_C$,\ncapped by the stall line', U_de_VC), ...
    'FontSize', 9, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center');

yline(0, 'k-', 'HandleVisibility', 'off');
text(V_D*1.01, n_pos, sprintf('$n^+ = %.2f$', n_pos), 'Color', orange, 'VerticalAlignment', 'bottom');
text(V_D*1.01, n_neg, sprintf('$n^- = %.2f$', n_neg), 'Color', orange, 'VerticalAlignment', 'top');

speed_labels = {'V_S', 'V_A', 'V_C', 'V_{NO}', 'V_{NE}', 'V_D'};
speed_vals   = [V_S, V_A, V_C, V_NO, V_NE, V_D];
[speed_vals, order] = sort(speed_vals);
speed_labels = speed_labels(order);
row_y = [n_neg - 0.3, n_neg - 0.7];
for i = 1:numel(speed_vals)
    xline(speed_vals(i), ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
    text(speed_vals(i), row_y(mod(i-1,2)+1), sprintf('$%s=%.1f$', speed_labels{i}, speed_vals(i)), ...
        'Color', [0.35 0.35 0.35], 'Interpreter', 'latex', 'FontSize', 9, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
end

xlabel('Equivalent Airspeed, (m/s)');
ylabel('load factor n (g)');
title('V-n Diagram: Maneuver Envelope and Gust Lines');
legend('Location', 'northwest');

%% ---- Console summary: critical values only ----
fprintf('\n--- V-n Diagram (Deliverable 2) ---\n');
fprintf('  Load factors:  n+ = %.2f    n- = %.2f    n_ult+ = %.2f    n_ult- = %.2f\n', n_pos, n_neg, n_ult_pos, n_ult_neg);
fprintf('  Key speeds:    V_A = %.1f m/s   V_C = %.1f m/s   V_D = %.1f m/s   V_NE = %.1f m/s\n', V_A, V_C, V_D, V_NE);
if NE_ok_A6
    fprintf('  Never-exceed check: V_NE > V_max,level (%.1f m/s) -- PASS\n', V_max_level);
else
    fprintf('  Never-exceed check: V_NE <= V_max,level (%.1f m/s) -- FAIL (see warning above)\n', V_max_level);
end
if gust_exceeds_n_pos
    fprintf('  >>> FINDING: stall-capped gust load factor at V_C (%.2f) exceeds the adopted n+ (%.2f) -- team decision needed (discussion Q1).\n', n_gust_VC_capped, n_pos);
end

%% Outputs; Carry the governing load factors and speeds forward for the rest of A8
outputs.VnDiagram = struct('n_pos_raw', n_pos_raw, 'n_pos_capped', n_pos_capped, 'n_pos', n_pos, ...
    'n_neg', n_neg, 'n_ult_pos', n_ult_pos, 'n_ult_neg', n_ult_neg, ...
    'V_S', V_S, 'V_A', V_A, 'V_C', V_C, 'V_NO', V_NO, 'V_NE', V_NE, 'V_D', V_D, ...
    'V_S_neg', V_S_neg, 'V_G', V_G, 'K_g', K_g, 'mu_g', mu_g, ...
    'n_gust_VC_raw', n_gust_VC_raw, 'n_gust_VC_capped', n_gust_VC_capped, ...
    'U_op', U_op, 'NE_ok_A6', NE_ok_A6);

params.performance.n_limit_pos = n_pos;
params.performance.n_limit_neg = n_neg;
params.performance.n_ult_pos   = n_ult_pos;
params.performance.n_ult_neg   = n_ult_neg;
params.performance.V_A  = V_A;
params.performance.V_NO = V_NO;
params.performance.V_NE = V_NE;
params.performance.V_D  = V_D;

end
