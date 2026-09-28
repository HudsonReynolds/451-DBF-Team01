function [outputs, params] = FuselageLoads(params)

% SizingParams Excel Sheet Contains Locations and Masses of Components for
% this analysis. 

%% Initializations

g = params.env.g;
rho = params.env.rho;
S = params.geometry.S_wing;
c = params.geometry.c_wing;

% Mass-Station Table -- the source of truth for structural analysis (team
% decision): its own sum IS the aircraft weight used throughout this
% file, matching WingLoads.m and TailLoads.m, rather than being scaled to
% match the iterated params.performance.MTOM. The two are compared and
% reported, not silently reconciled -- a large gap is a real finding.
massTable = readtable('SizingParams.xlsx', 'Sheet', 'MassBudget');
MTOM = sum(massTable.Mass_g)/1000; % kg
MTOW = MTOM * g; % N
x_cg_abs = sum(massTable.Mass_g .* massTable.Station_m) / sum(massTable.Mass_g);

x_LE_wing = params.structures.x_LE_wing;
x_ac_abs  = x_LE_wing + params.geometry.x_ac * c;
x_t_abs   = x_ac_abs + params.geometry.l_t;
x_cg_design_abs = x_LE_wing + params.geometry.x_cg_design * c;

fprintf('=== Fuselage: mass-station table (MassBudget tab, PLACEHOLDER except payload & servos) ===\n');
fprintf('  [mass table] MTOM = %.3f kg (source of truth) vs iterated params.performance.MTOM = %.3f kg (%.1f%% difference)\n', ...
    MTOM, params.performance.MTOM, 100*(MTOM-params.performance.MTOM)/params.performance.MTOM);
fprintf('  CG from the mass table: x = %.3f m; stability-assumed CG (x_cg_design): x = %.3f m (%.3f m difference)\n', ...
    x_cg_abs, x_cg_design_abs, x_cg_abs - x_cg_design_abs);
fprintf('  wing AC at x = %.3f m, tail AC at x = %.3f m\n', x_ac_abs, x_t_abs);

%% Computations

% MANEUVER AT V_A
n_pos = params.performance.n_limit_pos;
V_A = params.performance.V_A;
q_A = 0.5*rho*V_A^2;
Mac = q_A*S*c*params.aero.CM_ac_w;

Lt_man = (n_pos*MTOW*(x_cg_abs-x_ac_abs) + Mac) / (x_t_abs-x_ac_abs);
Lw_man = n_pos*MTOW - Lt_man;

fprintf('\n=== Fuselage: Case 1, maneuver at V_A=%.1f m/s (n=%+.2f) ===\n', V_A, n_pos);
fprintf('  Mac = q_A*S*c*Cm_ac,w = %.3f N*m (couple at the wing AC)\n', Mac);
fprintf('  Lw = %.2f N, Lt = %.2f N (Lw+Lt = %.2f N, check n*MTOW = %.2f N)\n', Lw_man, Lt_man, Lw_man+Lt_man, n_pos*MTOW);

% HARD LANDING, NO LIFT
sink_rate = params.structures.sink_rate;
k_gear    = params.structures.gear_stiffness;
n_L = sink_rate * sqrt(k_gear / (g*MTOW));
R = n_L*MTOW;

% Everything above (mass sum, CG, inertia loads, all three plots) is
% vectorized over the WHOLE table, so adding or removing rows there just
% works. These two lookups are the one place that needs specific rows by
% name -- if you rename or delete either, fail loudly here rather than
% silently propagating an empty/NaN result downstream:
nose_mask = strcmp(massTable.Component, 'Nose gear');
main_mask = strcmp(massTable.Component, 'Main gear (pair)');
if sum(nose_mask) ~= 1
    error('FuselageLoads:massTable', ['MassBudget tab must have exactly one row named ''Nose gear'' ' ...
        '(found %d) -- the hard-landing lever-rule split needs it by name.'], sum(nose_mask));
end
if sum(main_mask) ~= 1
    error('FuselageLoads:massTable', ['MassBudget tab must have exactly one row named ''Main gear (pair)'' ' ...
        '(found %d) -- the hard-landing lever-rule split needs it by name.'], sum(main_mask));
end
x_nose_gear = massTable.Station_m(nose_mask);
x_main_gear = massTable.Station_m(main_mask);
b_w = x_main_gear - x_nose_gear;
d_nose = x_cg_abs - x_nose_gear; 
d_main = x_main_gear - x_cg_abs;
F_main = R * d_nose / b_w; 
F_nose = R * d_main / b_w; 

fprintf('\n=== Fuselage: Case 2, hard landing (no lift) ===\n');
fprintf('  n_L = sink_rate*sqrt(k/(g*MTOW)) = %.1f*sqrt(%.0f/(%.2f*%.2f)) = %.2f g (sink_rate=%.2f m/s, k=%.0f N/m, ASSUMPTIONs)\n', ...
    sink_rate, k_gear, g, MTOW, n_L, sink_rate, k_gear);
fprintf('  R = n_L*MTOW = %.2f N; wheelbase b_w = %.3f m (nose @ %.3f m, main @ %.3f m)\n', R, b_w, x_nose_gear, x_main_gear);
fprintf('  F_nose = %.2f N (%.0f%%), F_main = %.2f N (%.0f%%) [F_nose+F_main = %.2f N, check R = %.2f N]\n', ...
    F_nose, 100*F_nose/R, F_main, 100*F_main/R, F_nose+F_main, R);

% BEAM: INTEGRATE FROM THE NOSE
x_max = max([massTable.Station_m; x_t_abs]) * 1.08;
X = linspace(0, x_max, 2000);

% Case 1 (maneuver)
F_inertia_man = -n_pos * (massTable.Mass_g/1000) * g;
stations_man  = [massTable.Station_m; x_ac_abs; x_t_abs];
forces_man    = [F_inertia_man; Lw_man; Lt_man];
V_man = zeros(size(X));
for i = 1:numel(stations_man)
    V_man = V_man + forces_man(i) * (X >= stations_man(i));
end
M_man = cumtrapz(X, V_man) + Mac * (X >= x_ac_abs);

% Case 2 (hard landing)
F_inertia_land = -n_L * (massTable.Mass_g/1000) * g;
stations_land  = [massTable.Station_m; x_nose_gear; x_main_gear];
forces_land    = [F_inertia_land; F_nose; F_main];
V_land = zeros(size(X));
for i = 1:numel(stations_land)
    V_land = V_land + forces_land(i) * (X >= stations_land(i));
end
M_land = cumtrapz(X, V_land);

M_envelope = max(abs(M_man), abs(M_land));
[M_env_max, i_env] = max(M_envelope);
governing_case = local_ternary(abs(M_man(i_env)) >= abs(M_land(i_env)), 'maneuver', 'hard landing');

fprintf('\n=== Fuselage: closure check (both cases must return to ~0 at the tail) ===\n');
fprintf('  Maneuver:    V(tail) = %.4f N, M(tail) = %.4f N*m\n', V_man(end), M_man(end));
fprintf('  Hard landing: V(tail) = %.4f N, M(tail) = %.4f N*m\n', V_land(end), M_land(end));
fprintf('  Design envelope: |M|_max = %.2f N*m at x = %.3f m (%s governs there)\n', M_env_max, X(i_env), governing_case);
fprintf('  Ultimate (1.5x limit): maneuver |M|_max = %.2f N*m, hard landing |M|_max = %.2f N*m\n', ...
    1.5*max(abs(M_man)), 1.5*max(abs(M_land)));

%% Plotting

blue = [0.00 0.45 0.74]; orange = [0.85 0.33 0.10];

figure('Name','Fuselage Loads','Color','w','Position',[100 100 950 900],'WindowStyle','docked');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

% Panel 1: applied loads for the maneuver case (inertia + reactions + couple).
nexttile; hold on; box on; grid on;
stem(massTable.Station_m, F_inertia_man, 'Color', orange, 'Marker', 'v', 'MarkerFaceColor', orange, 'MarkerSize', 5, 'DisplayName', 'inertia loads');
stem(x_ac_abs, Lw_man, 'Color', [0.20 0.60 0.30], 'LineWidth', 1.8, 'Marker', '^', 'MarkerFaceColor', [0.20 0.60 0.30], 'MarkerSize', 8, 'DisplayName', 'wing reaction, L_w');
stem(x_t_abs, Lt_man, 'Color', [0.70 0.20 0.60], 'LineWidth', 1.8, 'Marker', '^', 'MarkerFaceColor', [0.70 0.20 0.60], 'MarkerSize', 8, 'DisplayName', 'tail load, L_t');
yline(0, 'k-', 'HandleVisibility', 'off');
ylim([min(F_inertia_man)*1.3, max(Lw_man,Lt_man)*1.35]); % headroom above L_w so the M_ac label doesn't crowd the title
text(x_ac_abs, Lw_man*0.75, sprintf('  M_{ac}=%.2f N*m', Mac), 'FontSize', 8, 'Color', [0.35 0.35 0.35]);
xlabel('station from the nose, x (m)'); ylabel('point load (N)');
title(sprintf('Maneuver case at $V_A$=%.1f m/s, n=%+.2f: inertia loads, wing/tail reactions, couple at the wing', V_A, n_pos), 'Interpreter', 'latex');
legend('Location', 'best');
xlim([0, x_max]); % same range on all three panels so stations line up vertically

% Panel 2: shear, both cases overlaid.
nexttile; hold on; box on; grid on;
plot(X, V_man, '-', 'Color', blue, 'LineWidth', 1.6, 'DisplayName', sprintf('maneuver, n=%+.2f', n_pos));
plot(X, V_land, '--', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', sprintf('hard landing, n_L=%.1f, no lift', n_L));
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('station from the nose, x (m)'); ylabel('shear V (N)');
title('Shear force V(x): sum of the loads ahead of x -- returns to 0 at the tail');
legend('Location', 'best');
xlim([0, x_max]);

% Panel 3: bending moment, both cases plus the envelope.
nexttile; hold on; box on; grid on;
plot(X, M_man, '-', 'Color', blue, 'LineWidth', 1.6, 'DisplayName', sprintf('maneuver, n=%+.2f', n_pos));
plot(X, M_land, '--', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', sprintf('hard landing, n_L=%.1f', n_L));
plot(X, M_envelope, ':', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.4, 'DisplayName', 'envelope |M|');
plot(X(i_env), M_env_max, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6, 'HandleVisibility', 'off');
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('station from the nose, x (m)'); ylabel('bending moment M (N$\cdot$m)', 'Interpreter', 'latex');
title(sprintf('Bending moment M(x), with the design envelope: $|M|_{max}$=%.2f N*m at x=%.2f m (%s governs)', M_env_max, X(i_env), governing_case), 'Interpreter', 'latex');
legend('Location', 'best');
xlim([0, x_max]);

sgtitle(sprintf('Fuselage: Free-Free Beam, MTOM=%.2f kg, CG at x=%.3f m; ultimate 1.5$\\times$', MTOM, x_cg_abs), ...
    'Interpreter', 'latex', 'FontWeight', 'bold');

% Outputs
outputs.FuselageLoads.fuselage = struct( ...
    'mass_table_total_kg', MTOM, 'x_cg_abs', x_cg_abs, 'x_cg_design_abs', x_cg_design_abs, ...
    'x_ac_abs', x_ac_abs, 'x_t_abs', x_t_abs, ...
    'Lw_maneuver', Lw_man, 'Lt_maneuver', Lt_man, 'Mac_maneuver', Mac, ...
    'n_L', n_L, 'R_landing', R, 'F_nose', F_nose, 'F_main', F_main, ...
    'x_nose_gear', x_nose_gear, 'x_main_gear', x_main_gear, ...
    'M_envelope_max', M_env_max, 'M_envelope_station', X(i_env), 'governing_case', governing_case, ...
    'closure_V_maneuver', V_man(end), 'closure_M_maneuver', M_man(end), ...
    'closure_V_landing', V_land(end), 'closure_M_landing', M_land(end));

params.structures.fuselage_M_envelope_max = M_env_max;
params.structures.fuselage_M_envelope_station = X(i_env);
params.structures.fuselage_governing_case = governing_case;
params.structures.landing_n_L = n_L;
params.structures.landing_F_nose = F_nose;
params.structures.landing_F_main = F_main;
params.structures.x_cg_abs = x_cg_abs;
params.structures.mass_table_MTOM = MTOM; % source of truth for A8 structural analysis (D5-D8)
params.structures.mass_table_MTOW = MTOW;

end

function out = local_ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
