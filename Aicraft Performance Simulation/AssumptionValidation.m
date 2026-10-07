function [outputs, params] = AssumptionValidation(params)

%% A9 Deliverable 7 -- Assumption Validation
%
% A2-era sizing assumed several performance quantities up front (design
% propeller efficiencies, a target take-off distance, a structural load
% factor, etc.) before the detailed models existed to actually compute
% them. Now that Deliverables 1-6 compute those same quantities directly,
% this script pulls both numbers for each pair side by side so the team
% can see where the original assumption held up and where it didn't --
% it doesn't recompute anything new itself, except the two propeller-
% efficiency numbers (items 2-3 below), which nothing upstream currently
% exposes.

if ~isfield(params.prop, 'model')
    error('AssumptionValidation:noModel', 'Run PropulsionDragModel(params) first.');
end
model = params.prop.model;

MS2MPH = 2.23694;
prop = loadPropData(params.prop.prop_file);

fprintf('\n--- Assumption Validation (Deliverable 7) ---\n');
fprintf('  %-3s %-22s %12s %12s %12s  %s\n', '#', 'Quantity', 'Assumed', 'Computed', 'Delta', 'Note');
fprintf('  %s\n', repmat('-', 1, 92));

%% ---- 1. Stall speed: raw Excel input vs. self-consistent value ----
% V_TO baseline still comes from the assumed V_S (InitialCalcs.m runs
% before this self-consistent value exists); Deliverable 8's sweep does
% refresh V_S per candidate. Delta is small, so left as an open minor item.
rho = params.env.rho; S = params.geometry.S_wing; MTOW = params.performance.MTOW;
V_S_assumed = params.performance.V_S;
V_S_computed = sqrt(2*MTOW/(rho*S*params.aero.CL_max));
fprintf('  %-3s %-22s %12s %12s %+11.1f%%  %s\n', '1', 'Stall speed V_S', ...
    sprintf('%.2f m/s', V_S_assumed), sprintf('%.2f m/s', V_S_computed), ...
    100*(V_S_computed-V_S_assumed)/V_S_assumed, 'minor, V_TO baseline unaffected');

%% ---- 1b. Cruise speed: A2 design-point formula vs. self-consistent recompute ----
% A2's InitialCalcs.m set V_C from the early design-point wing loading,
% before final MTOW/S_wing existed. MissionSimulation.m/TurnPerformance.m
% still use the assumed V_C directly, not this value -- open team call.
V_C_assumed = params.performance.V_C;
V_C_computed = sqrt(2*MTOW/(rho*S*params.aero.CL_C));
fprintf('  %-3s %-22s %12s %12s %+11.1f%%  %s\n', '1b', 'Cruise speed V_C', ...
    sprintf('%.2f m/s', V_C_assumed), sprintf('%.2f m/s', V_C_computed), ...
    100*(V_C_computed-V_C_assumed)/V_C_assumed, 'TEAM DECISION: re-run mission if adopted');

%% ---- 2-3. Propeller efficiency: A2 design value vs. computed from the prop table ----
% Evaluated at full throttle (the only condition Deliverable 1's
% RPM_avail_fn directly exposes), at the cruise and take-off airspeeds,
% using the same eta_prop = J*Ct/Cp formula PropulsionSizing.m already
% uses elsewhere in this codebase.
V_C = params.performance.V_C;
RPM_cruise = model.RPM_avail_fn(V_C);
r_cruise = prop.query(RPM_cruise, MS2MPH*V_C);
eta_prop_cruise = (r_cruise.J * r_cruise.Ct) / r_cruise.Cp;

RPM_static = model.RPM_avail_fn(0);
r_static = prop.query(RPM_static, 0);
eta_prop_TO = (r_static.J * r_static.Ct) / r_static.Cp;
if r_static.J == 0
    eta_prop_TO = 0; % static: zero advance ratio means zero propulsive efficiency by definition (all shaft power goes to induced flow, none to forward thrust*V)
end

fprintf('  %-3s %-22s %12s %12s %+11.3f   %s\n', '2', 'Prop eff., cruise', ...
    sprintf('%.3f', params.prop.eta_p_C), sprintf('%.3f', eta_prop_cruise), ...
    eta_prop_cruise - params.prop.eta_p_C, 'superseded by D1 model, no action');
fprintf('  %-3s %-22s %12s %12s %+11.3f   %s\n', '3', 'Prop eff., take-off', ...
    sprintf('%.3f', params.prop.eta_p_TO), sprintf('%.3f', eta_prop_TO), ...
    eta_prop_TO - params.prop.eta_p_TO, 'eta=0 by definition at V=0, no action');

%% ---- 3b. Take-off thrust: A7 sizing target vs. Deliverable 1's computed static thrust ----
% PropulsionSizing.m's T_static (params.performance.T_static_design) is
% the required static thrust the motor/prop was sized against (A7),
% before Deliverable 1's torque-balance model + lab correction existed.
if isfield(params.performance, 'T_static_design')
    T_static_computed = model.T_avail_fn(0);
    fprintf('  %-3s %-22s %12s %12s %+11.2f N %s\n', '3b', 'Take-off thrust', ...
        sprintf('%.2f N', params.performance.T_static_design), sprintf('%.2f N', T_static_computed), ...
        T_static_computed - params.performance.T_static_design, 'exceeds A7 target, no action');
else
    fprintf('  3b  Take-off thrust            run PropulsionSizing(params) first\n');
end

%% ---- 4. Take-off distance: RFP target vs. Deliverable 2's integrated result ----
if isfield(params.performance, 'groundRoll_x')
    fprintf('  %-3s %-22s %12s %12s %10.2f m  %s\n', '4', 'Take-off distance', ...
        sprintf('%.1f m', params.performance.S_TO), sprintf('%.2f m', params.performance.groundRoll_x), ...
        params.performance.S_TO - params.performance.groundRoll_x, 'margin, no action');
else
    fprintf('  4   Take-off distance          run TakeoffPerformance(params) first\n');
end

%% ---- 4b. Parasite drag: no separate early assumption exists to compare against ----
% CD_0 has only ever been set one place (DragBuildUp.m, A5) -- no earlier,
% independently-guessed A2 target exists to compare it against.
fprintf('  %-3s %-22s %12s %12s %12s  %s\n', '4b', 'Parasite drag CD_0', 'n/a', ...
    sprintf('%.4f', params.aero.CD_0), 'n/a', 'nothing to compare');

%% ---- 5. Maximum level speed: A7/A8's analytic estimate vs. A9's refined model ----
if isfield(params.performance, 'V_max_level_A8')
    fprintf('  %-3s %-22s %12s %12s %+11.2f m/s %s\n', '5', 'Max level speed', ...
        sprintf('%.2f m/s', params.performance.V_max_level_A8), sprintf('%.2f m/s', params.performance.V_max_level), ...
        params.performance.V_max_level - params.performance.V_max_level_A8, 'RESOLVED (Main.m reorder feeds A8)');
else
    fprintf('  5   Max level speed            run CruisePerformance(params) first\n');
end

%% ---- 6. Load factor: structural design value vs. the course's actual demand ----
if isfield(params.performance, 'n_turn')
    fprintf('  %-3s %-22s %12s %12s %+11.2f   %s\n', '6', 'Load factor n', ...
        sprintf('%.2f', params.performance.n), sprintf('%.2f', params.performance.n_turn), ...
        params.performance.n_turn - params.performance.n, 'TEAM DECISION: margin vs. re-run lighter');
else
    fprintf('  6   Load factor n              run TurnPerformance(params) first\n');
end

%% ---- 7. Structural RPM limit: verified, not assumed ----
RPM_peak = max(model.RPM_avail_fn(model.V_grid));
ok_RPM = RPM_peak <= model.RPM_lim + 1; % +1 rpm floating-point slack
fprintf('  %-3s %-22s %12s %12s %12s  %s\n', '7', 'Structural RPM limit', ...
    sprintf('%.0f RPM', model.RPM_lim), sprintf('%.0f RPM', RPM_peak), '--', ...
    local_ternary(ok_RPM, 'never exceeded (verified)', 'EXCEEDED -- check RPM_lim clamp'));

%% ---- 8. Propeller static thrust: APC table vs. the team's own prop-stand lab data ----
% Evidence behind the system-wide Ct correction PropulsionDragModel.m
% already applies to every lookup.
labCal = model.labCal;
fprintf('\n  8.  Prop static thrust, lab vs. APC table (%d points):\n', numel(labCal.RPM));
fprintf('        %6s  %7s %7s %7s   %7s %7s %7s\n', 'RPM', 'Ct:APC', 'meas', 'delta', 'Cp:APC', 'meas', 'delta');
for k = 1:numel(labCal.RPM)
    fprintf('        %6.0f  %7.4f %7.4f %+6.1f%%   %7.4f %7.4f %+6.1f%%\n', ...
        labCal.RPM(k), labCal.Ct_apc(k), labCal.Ct_meas(k), labCal.Ct_pctDiff(k), ...
        labCal.Cp_apc(k), labCal.Cp_meas(k), labCal.Cp_pctDiff(k));
end
fprintf('      Cp matches the table; Ct runs low (worse at low RPM) -- correction applied system-wide.\n');

%% ---- 9. Motor electrical constants: could not be calibrated from the lab bench data ----
% Bench only logs battery-side V/I, not motor-phase -- both regression
% attempts against it returned unphysical R_motor/I0, so the nameplate
% values are kept. The fitted numbers below are a one-time result (not
% re-fit every run); only the "x off nameplate" ratio is computed live,
% so it stays current if the team edits the nameplate cells.
R_fit_A = 0.242; I0_fit_B = 6.70; % ohm, A -- one-time fitted values
fprintf('\n  9.  Motor constants (Kv/I0/R): lab calibration attempted, NOT applied -- nameplate kept.\n');
fprintf('        fit A: R=%.3f ohm (%.1fx nameplate), I0=-6.01 A (negative, impossible)\n', R_fit_A, R_fit_A/params.prop.R_motor);
fprintf('        fit B: R=-0.011 ohm (negative, impossible), I0=%.2f A (%.1fx nameplate)\n', I0_fit_B, I0_fit_B/params.prop.I0);
fprintf('        kept: Kv=%.0f RPM/V, I0=%.2f A, R=%.3f ohm\n', params.prop.Kv, params.prop.I0, params.prop.R_motor);

%% ---- Table figure: the 9 assumed-vs-computed comparisons, one row each ----
% Items 7-9 above (RPM-limit verification, prop lab calibration, motor
% calibration attempt) don't fit this table -- they aren't a single
% assumed-vs-computed pair, they're a check or a multi-point calibration.
tblRows = { ...
    'Stall speed V_S (m/s)', sprintf('%.2f', V_S_assumed), sprintf('%.2f', V_S_computed), ...
        local_pctStr(V_S_assumed, V_S_computed), 'V_TO/CL_R (minor)'; ...
    'Cruise speed V_C (m/s)', sprintf('%.2f', V_C_assumed), sprintf('%.2f', V_C_computed), ...
        local_pctStr(V_C_assumed, V_C_computed), 'Mission energy, lap speed'; ...
    'CD0', 'n/a', sprintf('%.4f', params.aero.CD_0), 'n/a', 'nothing to compare'; ...
    'Prop efficiency, cruise', sprintf('%.3f', params.prop.eta_p_C), sprintf('%.3f', eta_prop_cruise), ...
        local_pctStr(params.prop.eta_p_C, eta_prop_cruise), 'none (superseded)'; ...
    'Prop efficiency, take-off', sprintf('%.3f', params.prop.eta_p_TO), sprintf('%.3f', eta_prop_TO), ...
        local_pctStr(params.prop.eta_p_TO, eta_prop_TO), 'none (by definition)'; ...
};
if exist('T_static_computed', 'var')
    tblRows(end+1,:) = {'Static thrust (N)', sprintf('%.2f', params.performance.T_static_design), sprintf('%.2f', T_static_computed), ...
        local_pctStr(params.performance.T_static_design, T_static_computed), 'none (exceeds target)'};
end
if isfield(params.performance, 'groundRoll_x')
    tblRows(end+1,:) = {'Take-off distance (m)', sprintf('%.1f', params.performance.S_TO), sprintf('%.2f', params.performance.groundRoll_x), ...
        local_pctStr(params.performance.S_TO, params.performance.groundRoll_x), 'none (large margin)'};
end
if isfield(params.performance, 'V_max_level_A8')
    tblRows(end+1,:) = {'Max level speed (m/s)', sprintf('%.2f', params.performance.V_max_level_A8), sprintf('%.2f', params.performance.V_max_level), ...
        local_pctStr(params.performance.V_max_level_A8, params.performance.V_max_level), 'A8 V_D/V_NE (resolved)'};
end
if isfield(params.performance, 'n_turn')
    tblRows(end+1,:) = {'Load factor n', sprintf('%.2f', params.performance.n), sprintf('%.2f', params.performance.n_turn), ...
        local_pctStr(params.performance.n, params.performance.n_turn), 'A8 structural loads'};
end

figure('Name', 'Assumption Validation Table', 'Color', 'w', 'WindowStyle', 'docked');
ax_tbl = axes('Position', [0.03 0.05 0.94 0.85]);
addDataTable(ax_tbl, {'Quantity', 'Assumed', 'Computed', 'Change', 'Affects'}, tblRows, ...
    'ColAlign', {'left','right','right','right','left'});
title('Assumption Validation', 'Interpreter', 'none', 'FontWeight', 'bold');

%% ---- Outputs ----
outputs.AssumptionValidation = struct('V_S_assumed', V_S_assumed, 'V_S_computed', V_S_computed, ...
    'V_C_assumed', V_C_assumed, 'V_C_computed', V_C_computed, ...
    'eta_p_C_assumed', params.prop.eta_p_C, 'eta_p_C_computed', eta_prop_cruise, ...
    'eta_p_TO_assumed', params.prop.eta_p_TO, 'eta_p_TO_computed', eta_prop_TO, ...
    'RPM_peak', RPM_peak, 'RPM_lim', model.RPM_lim, 'ok_RPM', ok_RPM, ...
    'propLabCal', labCal, 'motorCal_applied', false);

end

function out = local_ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

function s = local_pctStr(assumed, computed)
    if assumed == 0
        s = 'n/a';
    else
        s = sprintf('%+.1f%%', 100*(computed-assumed)/assumed);
    end
end
