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
x_cg_abs = sum(massTable.Mass_g .* massTable.X_m) / sum(massTable.Mass_g);

x_LE_wing = params.structures.x_LE_wing;
x_ac_abs  = x_LE_wing + params.geometry.x_ac * c;
x_t_abs   = x_ac_abs + params.geometry.l_t;
x_cg_design_abs = x_LE_wing + params.geometry.x_cg_design * c;

%% Computations

% MANEUVER AT V_A
n_pos = params.performance.n_limit_pos;
V_A = params.performance.V_A;
q_A = 0.5*rho*V_A^2;
Mac = q_A*S*c*params.aero.CM_ac_w;

Lt_man = (n_pos*MTOW*(x_cg_abs-x_ac_abs) + Mac) / (x_t_abs-x_ac_abs);
Lw_man = n_pos*MTOW - Lt_man;

% HARD LANDING, NO LIFT
sink_rate = params.structures.sink_rate;
k_gear    = params.structures.gear_stiffness;
n_L = sink_rate * sqrt(k_gear / (g*MTOW));
R = n_L*MTOW;

% Tail-dragger hard landing: the fuselage beam must close (08_Structures_
% Beamer p.28, "a free-free beam MUST return shear and moment to zero at
% the free end... a non-zero closure is an error"), which needs two
% reaction points, the same as the tricycle nose/main split -- just with
% the tail wheel playing the nose's role as the second (now aft) point,
% i.e. both the main gear AND the tail wheel are treated as touching down
% together for this beam calculation. (Lecture 08B's "a tail-dragger
% lands on the mains, Fv = 1/2 n_L W" is a separate, more conservative
% simplification used specifically for sizing the main strut itself --
% it does not say the beam closure requirement is waived.)
%
% Everything above (mass sum, CG, inertia loads, all three plots) is
% vectorized over the WHOLE table, so adding or removing rows there just
% works. These two lookups are the one place that need specific rows by
% name -- if you rename or delete either, fail loudly here rather than
% silently propagating an empty/NaN result downstream:
main_mask = strcmp(massTable.Component, 'Main gear (pair)');
tailwheel_mask = strcmp(massTable.Component, 'Tail wheel');
if sum(main_mask) ~= 1
    error('FuselageLoads:massTable', ['MassBudget tab must have exactly one row named ''Main gear (pair)'' ' ...
        '(found %d) -- the hard-landing lever-rule split needs it by name.'], sum(main_mask));
end
if sum(tailwheel_mask) ~= 1
    error('FuselageLoads:massTable', ['MassBudget tab must have exactly one row named ''Tail wheel'' ' ...
        '(found %d) -- the hard-landing lever-rule split needs it by name.'], sum(tailwheel_mask));
end
x_main_gear = massTable.X_m(main_mask);
x_tailwheel = massTable.X_m(tailwheel_mask);
b_w = x_tailwheel - x_main_gear;
d_main = x_tailwheel - x_cg_abs;    % main's share scales with the OTHER point's arm (lever rule)
d_tail = x_cg_abs - x_main_gear;
F_main = R * d_main / b_w;
F_tailwheel = R * d_tail / b_w;

% BEAM: INTEGRATE FROM THE NOSE
x_max = max([massTable.X_m; x_t_abs]) * 1.08;
X = linspace(0, x_max, 2000);

% Case 1 (maneuver)
F_inertia_man = -n_pos * (massTable.Mass_g/1000) * g;
stations_man  = [massTable.X_m; x_ac_abs; x_t_abs];
forces_man    = [F_inertia_man; Lw_man; Lt_man];
V_man = zeros(size(X));
for i = 1:numel(stations_man)
    V_man = V_man + forces_man(i) * (X >= stations_man(i));
end
M_man = cumtrapz(X, V_man) + Mac * (X >= x_ac_abs);

% Case 2 (hard landing)
F_inertia_land = -n_L * (massTable.Mass_g/1000) * g;
stations_land  = [massTable.X_m; x_main_gear; x_tailwheel];
forces_land    = [F_inertia_land; F_main; F_tailwheel];
V_land = zeros(size(X));
for i = 1:numel(stations_land)
    V_land = V_land + forces_land(i) * (X >= stations_land(i));
end
M_land = cumtrapz(X, V_land);

M_envelope = max(abs(M_man), abs(M_land));
[M_env_max, i_env] = max(M_envelope);
governing_case = local_ternary(abs(M_man(i_env)) >= abs(M_land(i_env)), 'maneuver', 'hard landing');

% Shear envelope, determined independently of the bending envelope above
% -- the station and case that drive shear need not be the same ones
% that drive bending.
V_envelope = max(abs(V_man), abs(V_land));
[V_env_max, i_env_V] = max(V_envelope);
governing_case_V = local_ternary(abs(V_man(i_env_V)) >= abs(V_land(i_env_V)), 'maneuver', 'hard landing');

closure_tol = 0.05; % N*m -- both cases must return to ~0 at the tail (free-free beam)
if abs(M_man(end)) > closure_tol || abs(M_land(end)) > closure_tol
    warning('FuselageLoads:closure', ['Free-free beam did not close at the tail: maneuver M(tail) = %.4f N*m, ' ...
        'hard landing M(tail) = %.4f N*m.'], M_man(end), M_land(end));
end

fprintf('\n--- Fuselage Loads (Deliverable 3) ---\n');
fprintf('  Maneuver (V_A = %.1f m/s, n = %+.2f):   L_w = %.2f N, L_t = %.2f N\n', V_A, n_pos, Lw_man, Lt_man);
fprintf('  Hard landing (n_L = %.2f g):            F_main = %.2f N (%.1f N/wheel), F_tailwheel = %.2f N\n', n_L, F_main, F_main/2, F_tailwheel);
fprintf('  >>> GOVERNING CASE (bending): %s, |M|_max = %.2f N*m at x = %.3f m\n', governing_case, M_env_max, X(i_env));
fprintf('  >>> GOVERNING CASE (shear):   %s, |V|_max = %.2f N at x = %.3f m\n', governing_case_V, V_env_max, X(i_env_V));

%% Plotting

blue = [0.00 0.45 0.74]; orange = [0.85 0.33 0.10]; red = [0.80 0.00 0.00]; % red = driving/critical load marker

figure('Name','Fuselage Cases','Color','w','WindowStyle','docked');
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

% Panel 1: applied loads for the maneuver case (inertia + reactions + couple + CG).
nexttile; hold on; box on; grid on;
y_top = max(Lw_man,Lt_man)*1.6; % extra headroom vs before, for the CG label near the top
y_bot = min(F_inertia_man)*1.3;
ylim([y_bot, y_top]);
stem(massTable.X_m, F_inertia_man, 'Color', orange, 'Marker', 'v', 'MarkerFaceColor', orange, 'MarkerSize', 5, 'DisplayName', 'inertia loads');
stem(x_ac_abs, Lw_man, 'Color', [0.20 0.60 0.30], 'LineWidth', 1.8, 'Marker', '^', 'MarkerFaceColor', [0.20 0.60 0.30], 'MarkerSize', 8, 'DisplayName', 'wing reaction, L_w');
stem(x_t_abs, Lt_man, 'Color', [0.70 0.20 0.60], 'LineWidth', 1.8, 'Marker', '^', 'MarkerFaceColor', [0.70 0.20 0.60], 'MarkerSize', 8, 'DisplayName', 'tail load, L_t');
plot(x_cg_abs, 0, 'o', 'MarkerSize', 8, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w', 'LineWidth', 1.5, 'DisplayName', 'CG');
yline(0, 'k-', 'HandleVisibility', 'off');
text(x_ac_abs, Lw_man, sprintf('  $L_w$=%.1f N', Lw_man), 'FontSize', 8, 'Color', [0.20 0.60 0.30], 'VerticalAlignment', 'bottom');
text(x_t_abs, Lt_man, sprintf('  $L_t$=%.1f N', Lt_man), 'FontSize', 8, 'Color', [0.70 0.20 0.60], 'VerticalAlignment', 'bottom');
text(x_ac_abs, Lw_man*0.6, sprintf('  $M_{ac}$=%.2f N*m', Mac), 'FontSize', 8, 'Color', [0.35 0.35 0.35]);
text(x_cg_abs, y_top*0.93, sprintf('CG, x=%.3f m', x_cg_abs), 'FontSize', 8, 'Color', 'k', 'HorizontalAlignment', 'left');
xlabel('station from the nose, x (m)'); ylabel('point load (N)');
title(sprintf('Maneuver case at $V_A$=%.1f m/s, n=%+.2f: inertia loads, wing/tail reactions, couple at the wing, and CG', V_A, n_pos), 'Interpreter', 'latex');
legend('Location', 'best');
xlim([0, x_max]); % same range on both panels so stations line up vertically

% Panel 2: applied loads for the hard landing case (inertia + gear reactions, no lift).
% Tail-dragger, three-point touchdown: main gear and tail wheel both react
% (lever rule), so the beam closes -- see the note above the computation.
nexttile; hold on; box on; grid on;
y_top_L = max(F_main,F_tailwheel)*1.6;
y_bot_L = min(F_inertia_land)*1.3;
ylim([y_bot_L, y_top_L]);
stem(massTable.X_m, F_inertia_land, 'Color', orange, 'Marker', 'v', 'MarkerFaceColor', orange, 'MarkerSize', 5, 'DisplayName', 'inertia loads');
stem(x_main_gear, F_main, 'Color', [0.20 0.60 0.30], 'LineWidth', 1.8, 'Marker', '^', 'MarkerFaceColor', [0.20 0.60 0.30], 'MarkerSize', 8, 'DisplayName', 'main gear reaction, F_{main}');
stem(x_tailwheel, F_tailwheel, 'Color', [0.70 0.20 0.60], 'LineWidth', 1.8, 'Marker', '^', 'MarkerFaceColor', [0.70 0.20 0.60], 'MarkerSize', 8, 'DisplayName', 'tail-wheel reaction, F_{tailwheel}');
plot(x_cg_abs, 0, 'o', 'MarkerSize', 8, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w', 'LineWidth', 1.5, 'DisplayName', 'CG');
yline(0, 'k-', 'HandleVisibility', 'off');
text(x_main_gear, F_main, sprintf('  $F_{main}$=%.1f N (%.1f N/wheel)', F_main, F_main/2), 'FontSize', 8, 'Color', [0.20 0.60 0.30], 'VerticalAlignment', 'bottom');
text(x_tailwheel, F_tailwheel, sprintf('  $F_{tw}$=%.1f N', F_tailwheel), 'FontSize', 8, 'Color', [0.70 0.20 0.60], 'VerticalAlignment', 'bottom');
text(x_cg_abs, y_top_L*0.93, sprintf('CG, x=%.3f m', x_cg_abs), 'FontSize', 8, 'Color', 'k', 'HorizontalAlignment', 'left');
xlabel('station from the nose, x (m)'); ylabel('point load (N)');
title(sprintf('Hard landing case, $n_L$=%.2f g (no lift): inertia loads and gear reactions, three-point touchdown', n_L), 'Interpreter', 'latex');
legend('Location', 'best');
xlim([0, x_max]);

sgtitle(sprintf('Fuselage: Applied Loads by Case, MTOM=%.2f kg, CG at x=%.3f m', MTOM, x_cg_abs), ...
    'Interpreter', 'latex', 'FontWeight', 'bold');

% ---- Figure: shear and bending moment ----
figure('Name','Fuselage Loads','Color','w','WindowStyle','docked');
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

% Panel 1: shear, both cases plus the envelope.
nexttile; hold on; box on; grid on;
plot(X, V_man, '-', 'Color', blue, 'LineWidth', 1.6, 'DisplayName', sprintf('maneuver, n=%+.2f', n_pos));
plot(X, V_land, '--', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', sprintf('hard landing, n_L=%.1f, no lift', n_L));
plot(X, V_envelope, ':', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.4, 'DisplayName', 'envelope |V|');
plot(X(i_env_V), V_env_max, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('station from the nose, x (m)'); ylabel('shear V (N)');
title(sprintf('Shear force V(x), with the design envelope: $|V|_{max}$=%.2f N at x=%.2f m (%s governs)', V_env_max, X(i_env_V), governing_case_V), 'Interpreter', 'latex');
legend('Location', 'best');
xlim([0, x_max]);

% Panel 2: bending moment, both cases plus the envelope.
nexttile; hold on; box on; grid on;
plot(X, M_man, '-', 'Color', blue, 'LineWidth', 1.6, 'DisplayName', sprintf('maneuver, n=%+.2f', n_pos));
plot(X, M_land, '--', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', sprintf('hard landing, n_L=%.1f', n_L));
plot(X, M_envelope, ':', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.4, 'DisplayName', 'envelope |M|');
plot(X(i_env), M_env_max, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
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
    'n_L', n_L, 'R_landing', R, 'F_main', F_main, 'F_tailwheel', F_tailwheel, ...
    'x_main_gear', x_main_gear, 'x_tailwheel', x_tailwheel, ...
    'M_envelope_max', M_env_max, 'M_envelope_station', X(i_env), 'governing_case', governing_case, ...
    'V_envelope_max', V_env_max, 'V_envelope_station', X(i_env_V), 'governing_case_V', governing_case_V, ...
    'closure_V_maneuver', V_man(end), 'closure_M_maneuver', M_man(end), ...
    'closure_V_landing', V_land(end), 'closure_M_landing', M_land(end));

params.structures.fuselage_M_envelope_max = M_env_max;
params.structures.fuselage_M_envelope_station = X(i_env);
params.structures.fuselage_governing_case = governing_case;
params.structures.fuselage_V_envelope_max = V_env_max;
params.structures.fuselage_V_envelope_station = X(i_env_V);
params.structures.fuselage_governing_case_V = governing_case_V;
params.structures.landing_n_L = n_L;
params.structures.landing_F_main = F_main;
params.structures.landing_F_tailwheel = F_tailwheel;
params.structures.x_cg_abs = x_cg_abs;
params.structures.mass_table_MTOM = MTOM; % source of truth for A8 structural analysis (D5-D8)
params.structures.mass_table_MTOW = MTOW;

end

function out = local_ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
