function [outputs, params] = MissionSimulation(params)

%% A9 Deliverable 6 -- Mission Simulation
%
% Stitches Deliverable 2 (take-off), Deliverable 3 (climb), and the
% straight/turn flight-condition calculations Deliverable 4 and
% Deliverable 5 already use into the full competition mission profile --
% warm-up, ground roll, climb to the RFP altitude, the required laps,
% descent, and approach (A9's own segment list) -- tracking cumulative
% time and battery energy segment by segment. This is a new time-stepped
% model, not a re-run of VehicleWeightEstimation.m's older analytic
% weight-fraction energy budget (see the FINDING below for an important
% difference between them).
%
% ASSUMPTION, stated here per A9's explicit instruction to say whether
% acceleration or a constant speed is modelled: every segment flies at
% one constant speed (take-off/climb already integrate acceleration
% within themselves in Deliverables 2-3; this script reuses their
% already-computed total time/energy for those two segments rather than
% re-integrating, and treats every other segment -- straight, turn,
% descent, approach -- as constant-speed, no segment-to-segment
% acceleration modelled).
%
% Course layout and lap count are stated by the team's own requirements
% document, not a team design choice, so they're cited and hard-coded
% here rather than re-derived or added to SizingParams.xlsx -- same
% convention TakeoffPerformance.m already uses for mu_roll = 0.04:
%   - Team01_Requirements.slreqx, R3: "Aircraft shall complete 3 full laps
%     of the course portrayed in the RFP."
%   - Team01_StakeholderNeeds.slreqx, N2: "three full laps" (restates R3).
%   - Team01_Requirements.slreqx, R21: "shall not draw more than 1000W of
%     electrical power at any point during the complete mission profile"
%     -- the real RFP power limit. SizingParams.xlsx's P_limit_RFP is
%     still a 600W placeholder as of this script; this is checked against
%     the real 1000W value directly below, independent of that cell.
N_LAPS = 3;
P_RFP_LIMIT = 1000; % W, R21

if ~isfield(params.prop, 'model')
    error('MissionSimulation:noModel', 'Run PropulsionDragModel(params) first.');
end
model = params.prop.model;

if ~isfield(params.performance, 'groundRoll_t') || ~isfield(params.performance, 'groundRoll_V_LO')
    error('MissionSimulation:noTakeoff', 'Run TakeoffPerformance(params) first.');
end
if ~isfield(params.performance, 't_climb')
    error('MissionSimulation:noClimb', 'Run ClimbPerformance(params) first.');
end

rho = params.env.rho;
S = params.geometry.S_wing;
MTOW = params.performance.MTOW;
g = params.env.g;

%% ---- Warm-up, take-off, climb: reuse Deliverable 2 & 3's real results ----
t_TO = params.performance.groundRoll_t;
V_LO = params.performance.groundRoll_V_LO; %#ok<NASGU> % kept for outputs/debugging symmetry with V_ROC below
t_climb = params.performance.t_climb;
V_ROC = params.performance.V_ROC_max; %#ok<NASGU>

% Take-off and climb both fly at full throttle (Deliverables 2 and 3's
% own assumption), and Deliverable 1 already found the throttle cap binds
% continuously from V=0 m/s up through roughly 34 m/s -- both V_LO and
% V_ROC sit well inside that range, so electrical draw is exactly P_cap
% throughout both segments by construction (Pelec_avail_fn(V) is clamped
% to P_cap wherever the cap binds), not merely approximately -- no need
% to time-integrate a varying power here.
P_cap = model.P_cap;
E_TO = P_cap * t_TO;
E_climb = P_cap * t_climb;

% Warm-up: same "N equivalent take-offs' worth of energy" convention
% VehicleWeightEstimation.m already uses (params.performance.warmupN),
% now driven by Deliverable 2's actual integrated take-off time instead
% of that script's analytic weight-fraction shortcut.
warmupN = params.performance.warmupN;
t_warmup = warmupN * t_TO;
E_warmup = warmupN * E_TO;

%% ---- One lap: two straights (at V_C) + two 180-degree turns (at V_M) ----
V_C = params.performance.V_C;
lap_length = params.performance.lap_length;
R_req = params.performance.R_req;
V_M = params.performance.V_M;

q_C = 0.5*rho*V_C^2;
CL_C = min(MTOW/(q_C*S), params.aero.CL_max);
CD_C = model.CD_trim_fn(CL_C);
T_req_straight = q_C*S*CD_C;
T_avail_C = model.T_avail_fn(V_C);
if isnan(T_avail_C) || T_req_straight > T_avail_C
    error('MissionSimulation:noCruisePower', 'Cruise speed V_C = %.2f m/s is not achievable with this propulsion model.', V_C);
end
tau_C = model.find_throttle_for_thrust(V_C, T_req_straight);
P_elec_straight = model.Pelec_grid_fn(tau_C, V_C);

% "Charge each turn at its own load factor" (A9's own wording): load
% factor and CL are recomputed fresh at V_M/R_req rather than reused from
% Deliverable 5, exactly as that deliverable's own load factor would be
% for this weight -- there's only one course turn radius/speed, so this
% IS that turn's own load factor, applied identically to all 2*N_LAPS
% turns since neither weight nor geometry changes turn to turn.
n_turn = sqrt(1 + (V_M^2/(g*R_req))^2);
q_M = 0.5*rho*V_M^2;
CL_turn = n_turn*MTOW/(q_M*S);
CD_turn = model.CD_trim_fn(CL_turn);
T_req_turn = q_M*S*CD_turn;
T_avail_M = model.T_avail_fn(V_M);
if isnan(T_avail_M) || T_req_turn > T_avail_M
    error('MissionSimulation:noTurnPower', 'Turn speed V_M = %.2f m/s is not achievable with this propulsion model.', V_M);
end
tau_M = model.find_throttle_for_thrust(V_M, T_req_turn);
P_elec_turn = model.Pelec_grid_fn(tau_M, V_M);

t_straight_leg = lap_length / V_C;  % one of the two straights in a lap
t_turn_leg     = pi*R_req / V_M;    % one of the two 180-degree turns in a lap

t_lap = 2*t_straight_leg + 2*t_turn_leg;
E_lap = 2*P_elec_straight*t_straight_leg + 2*P_elec_turn*t_turn_leg;

%% ---- Descent and approach ----
% ASSUMPTION: both modelled as an unpowered glide (throttle = 0, no
% electrical draw) -- common practice for small electric RC aircraft
% returning to land, and nothing upstream in this codebase defines a
% descent throttle setting or an approach speed/distance to derive one
% from, so inventing a nonzero power figure here would be a guess
% dressed up as a number. Descent duration is set equal to the climb
% segment's time (symmetric assumption: the same altitude is lost as was
% gained, at a comparable rate); approach is a short fixed 5 s final
% segment before touchdown. Neither draws battery energy under this
% assumption, so they don't change the energy totals below, but they are
% still real mission-time segments and appear on the state-of-charge plot.
t_descent = t_climb;
E_descent = 0;
t_approach = 5;
E_approach = 0;

%% ---- Full mission: warm-up, take-off, climb, N_LAPS laps, descent, approach ----
t_mission = t_warmup + t_TO + t_climb + N_LAPS*t_lap + t_descent + t_approach;
E_mission = E_warmup + E_TO + E_climb + N_LAPS*E_lap + E_descent + E_approach;

E_usable = params.performance.usableBatteryEnergy_J;
energy_margin = E_usable - E_mission;
ok_energy = energy_margin >= 0;

P_peak = max([P_cap, P_elec_straight, P_elec_turn]);
ok_power_limit = P_peak <= P_RFP_LIMIT;

% Laps possible: how many laps the usable energy supports after reserving
% warm-up, take-off, climb, descent, and approach (all of which still
% have to happen regardless of lap count) -- not just whether the
% required 3 fit.
E_for_laps = E_usable - E_warmup - E_TO - E_climb - E_descent - E_approach;
laps_possible = floor(max(E_for_laps, 0) / E_lap);

% Energy left after flying exactly the required mission, and the battery
% mass it represents (same energy/rho_battery relationship
% VehicleWeightEstimation.m already uses for the pack itself).
energy_left_after_mission = E_usable - E_mission;
battery_mass_left_kg = energy_left_after_mission / params.prop.rho_battery;

%% ---- Console ----
fprintf('\n--- Mission Simulation (Deliverable 6) ---\n');
fprintf('  Segment speeds:       constant per segment (no inter-segment acceleration modelled; take-off/climb already\n');
fprintf('                        integrate their own acceleration internally in Deliverables 2-3)\n');
fprintf('  Laps required:        %d  (R3 / N2, stated by the requirements, not a team choice)\n', N_LAPS);
fprintf('  Warm-up:              t = %.2f s, E = %.1f kJ  (%d x take-off)\n', t_warmup, E_warmup/1000, warmupN);
fprintf('  Take-off:             t = %.2f s, E = %.1f kJ\n', t_TO, E_TO/1000);
fprintf('  Climb:                t = %.2f s, E = %.1f kJ\n', t_climb, E_climb/1000);
fprintf('  Straight leg:         t = %.2f s, P = %.1f W  (at V_C = %.2f m/s)\n', t_straight_leg, P_elec_straight, V_C);
fprintf('  Turn leg:             t = %.2f s, P = %.1f W  (at V_M = %.2f m/s, n = %.2f)\n', t_turn_leg, P_elec_turn, V_M, n_turn);
fprintf('  One lap:              t = %.2f s, E = %.1f kJ\n', t_lap, E_lap/1000);
fprintf('  Descent:               t = %.2f s, E = %.1f kJ  (unpowered glide, ASSUMPTION)\n', t_descent, E_descent/1000);
fprintf('  Approach:              t = %.2f s, E = %.1f kJ  (unpowered glide, ASSUMPTION)\n', t_approach, E_approach/1000);
fprintf('  Mission total time:   %.1f s  (%.1f min)\n', t_mission, t_mission/60);
fprintf('  Mission total energy: %.1f kJ\n', E_mission/1000);
fprintf('  Usable battery energy: %.1f kJ\n', E_usable/1000);
fprintf('  Energy margin:        %.1f kJ -- %s\n', energy_margin/1000, local_ternary(ok_energy, 'PASS', 'FAIL'));
fprintf('  Peak electrical power: %.1f W vs RFP limit %.0f W (R21) -- %s\n', P_peak, P_RFP_LIMIT, local_ternary(ok_power_limit, 'PASS', 'FAIL'));
fprintf('  Laps possible:         %d  vs %d required -- %s\n', laps_possible, N_LAPS, local_ternary(laps_possible >= N_LAPS, 'PASS', 'FAIL'));
fprintf('  Energy left after mission: %.1f kJ = %.3f kg of battery\n', energy_left_after_mission/1000, battery_mass_left_kg);
if abs(params.performance.P_limit_RFP - P_RFP_LIMIT) > 1
    fprintf('  >>> NOTE: SizingParams.xlsx''s P_limit_RFP is still %.0f W (a placeholder); R21''s real value is %.0f W -- update the spreadsheet. This script already checks against the real number regardless of that cell.\n', ...
        params.performance.P_limit_RFP, P_RFP_LIMIT);
end

% FINDING: VehicleWeightEstimation.m's own battery sizing -- the one that
% actually set MTOM/MTOW for the whole pipeline -- computes T_LF =
% 2*lap_length/V_C and T_TU = 2*pi*R_req/V_M, which is structurally ONE
% lap's worth of straight+turn flight (two straights, two turns summing
% to one full circle), and sums it into the battery weight fraction
% exactly once. It is never multiplied by the number of laps the course
% actually requires. This script's mission total above already accounts
% for all 3 laps; VehicleWeightEstimation.m's sizing appears not to.
fprintf('  >>> FINDING: VehicleWeightEstimation.m''s battery sizing (which set the MTOM/MTOW used throughout the whole\n');
fprintf('      pipeline) computes straight+turn energy for ONE lap and never multiplies by the %d laps R3 requires --\n', N_LAPS);
fprintf('      worth the team''s attention; the battery (and therefore MTOM) may be undersized for the real mission.\n');

%% ---- Plot: battery state of charge through the mission, segments shaded ----
blue = [0.00 0.45 0.74]; red = [0.80 0.00 0.00]; gray = [0.4 0.4 0.4];

seg_name = {'Warm-up', 'Ground roll', 'Climb'};
seg_t    = [t_warmup, t_TO, t_climb];
seg_E    = [E_warmup, E_TO, E_climb];
for lap = 1:N_LAPS
    seg_name = [seg_name, {sprintf('Lap %d', lap)}]; %#ok<AGROW>
    seg_t    = [seg_t, t_lap]; %#ok<AGROW>
    seg_E    = [seg_E, E_lap]; %#ok<AGROW>
end
seg_name = [seg_name, {'Descent', 'Approach'}];
seg_t    = [seg_t, t_descent, t_approach];
seg_E    = [seg_E, E_descent, E_approach];

t_points = [0, cumsum(seg_t)];
SOC_pct = 100 * (E_usable - [0, cumsum(seg_E)]) / E_usable;
i_laps_end = 1 + 3 + N_LAPS; % index into t_points/SOC_pct at the end of the required laps

figure('Name','Mission State of Charge','Color','w','WindowStyle','docked');
hold on; box on; grid on;

% Shade each segment and label it along the top, matching A9's example.
shadeColors = repmat([0.93 0.93 0.93; 1 1 1], ceil(numel(seg_name)/2), 1);
yl_fixed = [min(0, SOC_pct(end))-5, 108];
for k = 1:numel(seg_name)
    xregion(t_points(k), t_points(k+1), 'FaceColor', shadeColors(k,:), 'FaceAlpha', 1, 'HandleVisibility', 'off');
    xc = 0.5*(t_points(k)+t_points(k+1));
    text(xc, yl_fixed(2)-1, seg_name{k}, 'FontSize', 8, 'Color', gray, 'Rotation', 90, ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'top');
end

plot(t_points, SOC_pct, '-', 'Color', blue, 'LineWidth', 2.0, 'DisplayName', 'State of charge');
yline(0, '--', 'Color', red, 'LineWidth', 1.2, 'DisplayName', 'Depleted');
xline(t_points(i_laps_end), ':', 'Color', gray, 'LineWidth', 1.2, 'DisplayName', 'End of required laps');
plot(t_points(i_laps_end), SOC_pct(i_laps_end), 'o', 'MarkerFaceColor', local_ternary(ok_energy, [0.85 0.33 0.10], red), ...
    'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
text(t_points(i_laps_end), SOC_pct(i_laps_end), sprintf('  %.0f%% at %.0f s', SOC_pct(i_laps_end), t_points(i_laps_end)), ...
    'FontSize', 9, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', 'Interpreter', 'none');

xlabel('Mission time (s)');
ylabel('State of charge (%)', 'Interpreter', 'none');
title('Mission state of charge: warm-up, ground roll, climb, 3 laps, descent, approach');
legend('Location', 'southwest');
xlim([0, t_points(end)]);
ylim(yl_fixed);

%% ---- Outputs ----
outputs.MissionSimulation = struct('N_LAPS', N_LAPS, 't_warmup', t_warmup, 'E_warmup', E_warmup, ...
    't_TO', t_TO, 'E_TO', E_TO, 't_climb', t_climb, 'E_climb', E_climb, ...
    't_straight_leg', t_straight_leg, 'P_elec_straight', P_elec_straight, ...
    't_turn_leg', t_turn_leg, 'P_elec_turn', P_elec_turn, 'n_turn', n_turn, ...
    't_lap', t_lap, 'E_lap', E_lap, 't_descent', t_descent, 'E_descent', E_descent, ...
    't_approach', t_approach, 'E_approach', E_approach, ...
    't_mission', t_mission, 'E_mission', E_mission, ...
    'E_usable', E_usable, 'energy_margin', energy_margin, 'ok_energy', ok_energy, ...
    'P_peak', P_peak, 'P_RFP_limit', P_RFP_LIMIT, 'ok_power_limit', ok_power_limit, ...
    'laps_possible', laps_possible, 'energy_left_after_mission', energy_left_after_mission, ...
    'battery_mass_left_kg', battery_mass_left_kg, ...
    't_points', t_points, 'SOC_pct', SOC_pct);

params.performance.mission_t = t_mission;
params.performance.mission_E = E_mission;
params.performance.mission_ok_energy = ok_energy;
params.performance.mission_ok_power = ok_power_limit;
params.performance.mission_laps_possible = laps_possible;
params.performance.mission_E_left = energy_left_after_mission;
params.performance.mission_battery_mass_left_kg = battery_mass_left_kg;

end

function out = local_ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
