function [outputs, params] = VnDiagram(params)

% A8 Deliverable 2 -- V-n diagram.
%
% Limit maneuvering load factors: FAR 23.337 (pre-2017 amendment), normal
% category. Ultimate factor: FAR 23.303 (1.5x limit). Gust load factor:
% FAR 23.341, gust velocities from FAR 23.333(c) at sea level. Dive speed:
% simplified FAR 23.335(b) (the wing-loading chart refinement is not
% implemented -- this uses the two governing multiples of V_C and of the
% max level speed from A6/PropulsionSizing).
%
% CL_max_neg is not a measured quantity (Sheet1 carries an assumed value,
% flagged there) -- refine it with an XFLR5 negative-alpha sweep.

%% ---- Unit conversions (FAR 23.341's gust formula is defined in US units) ----
KG2LB      = 2.20462;   % kg -> lbm (numerically lbf at 1g, standard aviation usage)
M2FT       = 3.28084;   % m -> ft
MS2KT      = 1.94384;   % m/s -> knots
KGM3_2_SLUGFT3 = 0.00194032; % kg/m^3 -> slug/ft^3

%% ---- Aircraft-specific values (pulled from params -- Sheet1-linked or computed upstream) ----
MTOM   = params.performance.MTOM;      % kg
MTOW   = params.performance.MTOW;      % N
S      = params.geometry.S_wing;       % m^2
c_bar  = params.geometry.c_wing;       % m, mean chord (rectangular-wing approximation, matches the rest of the codebase)
CL_max = params.aero.CL_max;
CL_max_neg = params.aero.CL_max_neg;   % ASSUMPTION -- see Sheet1 note
CL_alpha = params.aero.CL_Alpha;       % per rad, whole-aircraft (from AircraftScissorPlot via A5B)
rho    = params.env.rho;
g      = params.env.g;

V_S    = params.performance.V_S;       % m/s, positive 1g stall speed (Sheet1 input)
V_C    = params.performance.V_C;       % m/s, design cruise speed (from InitialCalcs)
V_max_level = params.performance.V_max_level; % m/s, max level speed at 100% throttle (from A6/PropulsionSizing)

%% ---- FAR 23.337: limit maneuvering load factors ----
W_lb = MTOM * KG2LB;
n_pos_raw = 2.1 + 24000/(W_lb + 10000);
n_pos = min(max(n_pos_raw, 2.5), 3.8);          % FAR 23.337(a), normal category bounds
n_neg = -0.4*n_pos;                              % FAR 23.337(b)

n_ult_pos = 1.5*n_pos;                           % FAR 23.303, ultimate = 1.5x limit
n_ult_neg = 1.5*n_neg;

%% ---- Key speeds ----
V_A = V_S*sqrt(n_pos);                                             % positive maneuvering speed
V_D = max(1.25*V_C, 1.15*V_max_level);                              % simplified FAR 23.335(b)
V_NE = 0.9*V_D;                                                     % FAR 23.1505

V_S_neg = sqrt(2*MTOW/(rho*S*abs(CL_max_neg)));                     % negative 1g stall speed
V_G = V_S_neg*sqrt(abs(n_neg));                                     % speed where the negative stall curve reaches n_neg

%% ---- FAR 23.341: gust load factor ----
W_S_psf = (MTOM*KG2LB) / (S*M2FT^2);          % wing loading, psf
rho_slug = rho*KGM3_2_SLUGFT3;
c_ft = c_bar*M2FT;
g_ft = g*M2FT;

mu_g = 2*W_S_psf / (rho_slug*c_ft*g_ft*CL_alpha);
K_g = 0.88*mu_g / (5.3+mu_g);

U_de_VC = 50; % ft/s, FAR 23.333(c), sea level
U_de_VD = 25; % ft/s, FAR 23.333(c), sea level

gustN = @(U_de_fps, V_ms) 1 + (K_g*U_de_fps*(V_ms*MS2KT)*CL_alpha) / (498*W_S_psf);

% Gust load actually achieved at V_C, capped by the stall boundary (the
% wing stalls before it can reach the raw gust-line load):
n_gust_VC_raw = gustN(U_de_VC, V_C);
n_stall_VC = 0.5*rho*V_C^2*S*CL_max/MTOW;
n_gust_VC_capped = min(n_gust_VC_raw, n_stall_VC);

n_design = max(n_pos, n_gust_VC_capped); % whichever governs -- maneuver limit or the physically achievable (stall-capped) gust at cruise

% Operational gust: a smaller, non-certification design gust used for
% preliminary sizing on an aircraft this size -- design/judgment choice,
% not linked to a Sheet1 parameter (same treatment as the servo safety
% margin in ServoSizing.m). ADJUST AND JUSTIFY for your team's design.
U_op_ms = 5; % m/s
gustN_op = @(V_ms) 1 + (K_g*(U_op_ms/0.3048)*(V_ms*MS2KT)*CL_alpha) / (498*W_S_psf);
try
    V_op_limit = fzero(@(V) gustN_op(V) - n_pos, [0.1, 5*V_D]); % where the operational gust line meets the positive limit
catch
    V_op_limit = V_D; % operational gust doesn't reach n^+ within a reasonable range -- draw it across the full envelope instead
end

%% ---- Never-exceed check against A6 (and A9, once available) ----
NE_ok_A6 = V_NE > V_max_level;
fprintf('=== V-n diagram: never-exceed check ===\n');
if NE_ok_A6
    fprintf('  V_NE = %.1f m/s exceeds the max level speed from A6 (%.1f m/s) -- OK.\n', V_NE, V_max_level);
else
    warning('V_NE (%.1f m/s) does NOT exceed the max level speed from A6 (%.1f m/s) -- V_D must increase.', V_NE, V_max_level);
end
fprintf('  A9 max level speed not available yet -- recheck once A9 is complete.\n');

%% ---- Envelope boundary (for the solid/shaded flight envelope) ----
V_pos_curve = linspace(0, V_A, 300);
n_pos_curve = 0.5*rho*V_pos_curve.^2*S*CL_max/MTOW;

V_neg_curve = linspace(0, V_G, 300);
n_neg_curve = -0.5*rho*V_neg_curve.^2*S*abs(CL_max_neg)/MTOW;

% Closed boundary polygon for shading: origin -> positive stall curve -> top
% limit -> right edge at V_D -> bottom limit -> negative stall curve -> origin.
V_poly = [V_pos_curve, V_D, V_D, fliplr(V_neg_curve)];
n_poly = [n_pos_curve, n_pos, n_neg, fliplr(n_neg_curve)];

%% ---- Plot ----
blue  = [0.00 0.45 0.70];  % Okabe-Ito colorblind-safe palette
orange= [0.90 0.35 0.00];
teal  = [0.00 0.60 0.50];

figure('Name','V-n Diagram','Color','w','Position',[100 100 900 600],'WindowStyle','docked'); hold on; box on; grid on;

fill(V_poly, n_poly, blue, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');

% Positive stall boundary: dotted below the stall speed, solid above it.
plot(linspace(0, V_S, 50), 0.5*rho*linspace(0,V_S,50).^2*S*CL_max/MTOW, ':', 'Color', blue, 'HandleVisibility', 'off');
plot(linspace(V_S, V_A, 200), 0.5*rho*linspace(V_S,V_A,200).^2*S*CL_max/MTOW, '-', 'Color', blue, 'LineWidth', 1.5, 'DisplayName', 'stall boundary');

% Negative stall boundary: dotted below V_S_neg, solid from V_S_neg to V_G.
plot(linspace(0, V_S_neg, 50), -0.5*rho*linspace(0,V_S_neg,50).^2*S*abs(CL_max_neg)/MTOW, ':', 'Color', blue, 'HandleVisibility', 'off');
plot(linspace(V_S_neg, V_G, 200), -0.5*rho*linspace(V_S_neg,V_G,200).^2*S*abs(CL_max_neg)/MTOW, '-', 'Color', blue, 'LineWidth', 1.5, 'HandleVisibility', 'off');

% Limit load factor lines: top (V_A to V_D), right edge (V_D), bottom (V_G to V_D, held to the dive speed).
plot([V_A V_D], [n_pos n_pos], '-', 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'limit load factors');
plot([V_D V_D], [n_neg n_pos], '-', 'Color', orange, 'LineWidth', 1.5, 'HandleVisibility', 'off');
plot([V_G V_D], [n_neg n_neg], '-', 'Color', orange, 'LineWidth', 1.5, 'HandleVisibility', 'off');

% Raw FAR 23 gust lines (positive and negative), drawn full length for reference:
V_gust = linspace(0, V_D, 200);
plot(V_gust, gustN(U_de_VC, V_gust), '--', 'Color', teal, 'DisplayName', sprintf('FAR 23 gust, %d ft/s at V_C (raw)', U_de_VC));
plot(V_gust, 2 - gustN(U_de_VC, V_gust), '--', 'Color', teal, 'HandleVisibility', 'off');
plot(V_gust, gustN(U_de_VD, V_gust), '-.', 'Color', teal, 'DisplayName', sprintf('FAR 23 gust, %d ft/s at V_D (raw)', U_de_VD));
plot(V_gust, 2 - gustN(U_de_VD, V_gust), '-.', 'Color', teal, 'HandleVisibility', 'off');

% Operational (design) gust line, drawn up to where it meets the positive limit:
V_op_line = linspace(0, V_op_limit, 100);
plot(V_op_line, gustN_op(V_op_line), '-', 'Color', [0.9 0.6 0.0], 'LineWidth', 1.3, 'DisplayName', sprintf('operational gust, %.0f m/s (design)', U_op_ms));

% Mark the gust load at V_C, capped by the stall line:
plot(V_C, n_gust_VC_capped, 'd', 'Color', teal, 'MarkerFaceColor', teal, 'MarkerSize', 8, 'HandleVisibility', 'off');
text(V_C, n_gust_VC_capped, sprintf('  %d ft/s gust at V_C,\ncapped by the stall line', U_de_VC), 'FontSize', 9, 'VerticalAlignment', 'middle');

yline(0, 'k-', 'HandleVisibility', 'off');
text(V_D*1.01, n_pos, sprintf('n^+ = %.2f', n_pos), 'Color', orange, 'VerticalAlignment', 'bottom');
text(V_D*1.01, n_neg, sprintf('n^- = %.2f', n_neg), 'Color', orange, 'VerticalAlignment', 'top');

% Label V_S, V_A, V_C, V_NE and V_D directly on the plot with xline's native
% label (sits right on the line, so it can't be missed or drift out of sync
% with the curve). Alternate top/bottom so labels don't collide when two
% speeds land close together.
speed_labels = {'V_S', 'V_A', 'V_C', 'V_{NE}', 'V_D'};
speed_vals   = [V_S, V_A, V_C, V_NE, V_D];
[speed_vals, order] = sort(speed_vals);
speed_labels = speed_labels(order);
valign = {'bottom', 'top'};
for i = 1:numel(speed_vals)
    xline(speed_vals(i), ':', sprintf('$%s = %.1f$', speed_labels{i}, speed_vals(i)), ...
        'Color', [0.35 0.35 0.35], 'Interpreter', 'latex', 'FontSize', 9, ...
        'LabelVerticalAlignment', valign{mod(i-1,2)+1}, 'LabelHorizontalAlignment', 'right', ...
        'HandleVisibility', 'off');
end

xlabel('equivalent airspeed V_{EAS} (m/s)');
ylabel('load factor n (g)');
title('V-n Diagram: Maneuver Envelope and Gust Lines');
legend('Location', 'northwest');
xlim([0, V_D*1.08]);
ylim([n_neg*1.4, n_pos*1.25]);

%exportgraphics(gcf, 'Vn_diagram.png', 'Resolution', 300);

%% ---- Assumptions / equations table ----
fprintf('\n=== V-n diagram: equations, constants and aircraft-specific values ===\n');
fprintf('  FAR 23.337(a)  n^+ = clip(2.1 + 24000/(W_lb+10000), 2.5, 3.8), W = %.2f lb  ->  n^+ = %.3f\n', W_lb, n_pos);
fprintf('  FAR 23.337(b)  n^- = -0.4 n^+  ->  n^- = %.3f\n', n_neg);
fprintf('  FAR 23.303     ultimate = 1.5x limit  ->  n_ult^+ = %.3f, n_ult^- = %.3f\n', n_ult_pos, n_ult_neg);
fprintf('  V_A = V_S sqrt(n^+) = %.2f m/s   (V_S = %.2f m/s)\n', V_A, V_S);
fprintf('  V_C (from A2/InitialCalcs)      = %.2f m/s\n', V_C);
fprintf('  V_D = max(1.25 V_C, 1.15 V_max) = %.2f m/s   (V_max level, A6 = %.2f m/s)\n', V_D, V_max_level);
fprintf('  V_NE = 0.9 V_D                  = %.2f m/s\n', V_NE);
fprintf('  FAR 23.341 gust: n = 1 +/- Kg*Ude*V[kt]*a/(498*W/S[psf])\n');
fprintf('    mu_g = 2(W/S)/(rho*c_bar*g*a) = %.3f,  K_g = 0.88 mu_g/(5.3+mu_g) = %.3f\n', mu_g, K_g);
fprintf('    U_de = %d ft/s at V_C, %d ft/s at V_D (FAR 23.333(c), sea level)\n', U_de_VC, U_de_VD);
fprintf('    n_gust(V_C) raw = %.3f (exceeds CL_max -- not physically achievable), stall-capped = %.3f   vs.  n^+ = %.3f  ->  design n = %.3f (%s governs)\n', ...
    n_gust_VC_raw, n_gust_VC_capped, n_pos, n_design, ternary(n_design==n_pos, 'maneuver', 'stall-capped gust'));
fprintf('    operational (design) gust adopted = %.1f m/s, meets n^+ at V = %.2f m/s\n', U_op_ms, V_op_limit);
fprintf('  Atmospheric/aircraft constants: rho = %.3f kg/m^3, g = %.2f m/s^2, S = %.4f m^2, c_bar = %.3f m\n', rho, g, S, c_bar);
fprintf('    CL_max = %.3f, CL_max_neg = %.2f (ASSUMPTION), CL_alpha = %.3f /rad, MTOM = %.2f kg\n', CL_max, CL_max_neg, CL_alpha, MTOM);

%% ---- Outputs ----
outputs.VnDiagram = struct('n_pos', n_pos, 'n_neg', n_neg, 'n_ult_pos', n_ult_pos, 'n_ult_neg', n_ult_neg, ...
    'V_S', V_S, 'V_A', V_A, 'V_C', V_C, 'V_D', V_D, 'V_NE', V_NE, 'V_S_neg', V_S_neg, 'V_G', V_G, ...
    'K_g', K_g, 'mu_g', mu_g, 'n_gust_VC_raw', n_gust_VC_raw, 'n_gust_VC_capped', n_gust_VC_capped, ...
    'n_design', n_design, 'U_op_ms', U_op_ms, 'V_op_limit', V_op_limit, 'NE_ok_A6', NE_ok_A6);

% Carry the governing load factors and speeds forward for the rest of A8
% (wing/tail/fuselage loads all need n_design and V_A/V_D):
params.performance.n_limit_pos = n_pos;
params.performance.n_limit_neg = n_neg;
params.performance.n_ult_pos   = n_ult_pos;
params.performance.n_ult_neg   = n_ult_neg;
params.performance.n_design    = n_design;
params.performance.V_A = V_A;
params.performance.V_D = V_D;
params.performance.V_NE = V_NE;

end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
