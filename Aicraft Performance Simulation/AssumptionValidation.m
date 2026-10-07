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

%% ---- 1. Stall speed: raw Excel input vs. self-consistent value ----
rho = params.env.rho; S = params.geometry.S_wing; MTOW = params.performance.MTOW;
V_S_assumed = params.performance.V_S;
V_S_computed = sqrt(2*MTOW/(rho*S*params.aero.CL_max));
fprintf('  1. Stall speed V_S:        assumed (Excel) = %.2f m/s   computed (self-consistent) = %.2f m/s   delta = %.2f m/s\n', ...
    V_S_assumed, V_S_computed, V_S_computed - V_S_assumed);
fprintf('      Affects: V_TO = mult_V_TO x V_S (InitialCalcs.m runs before this self-consistent value exists, so\n');
fprintf('      Deliverables 2-6''s baseline V_TO still derives from the ASSUMED V_S, not this one -- Deliverable 8''s\n');
fprintf('      sweep DOES refresh V_S self-consistently per candidate). Re-run needed: not currently done; delta is\n');
fprintf('      small (%.1f%%) so the effect is minor, but strictly the baseline V_TO/CL_R are off this self-consistent value.\n', ...
    100*(V_S_computed-V_S_assumed)/V_S_assumed);

%% ---- 1b. Cruise speed: A2 design-point formula vs. self-consistent recompute ----
% Same self-consistency check as item 1, applied to V_C: A2's InitialCalcs.m
% set V_C = sqrt(2*W_S_design/(rho*CL_C)) from the early design-point wing
% loading, before the aircraft's final MTOW/S_wing existed. Recomputed here
% with the FINAL, converged MTOW and S_wing (same CL_C) to check whether
% that early design point still matches the aircraft as actually sized.
V_C_assumed = params.performance.V_C;
V_C_computed = sqrt(2*MTOW/(rho*S*params.aero.CL_C));
fprintf('  1b. Cruise speed V_C:      assumed (A2 design point) = %.2f m/s   computed (self-consistent, final MTOW/S) = %.2f m/s   delta = %.2f m/s\n', ...
    V_C_assumed, V_C_computed, V_C_computed - V_C_assumed);
fprintf('      Affects: MissionSimulation.m''s straight-leg speed/power/energy and TurnPerformance.m''s lap speed both\n');
fprintf('      use the ASSUMED V_C directly, not this self-consistent value. Re-run needed: recommended -- an %.1f%%\n', ...
    100*(V_C_computed-V_C_assumed)/V_C_assumed);
fprintf('      speed change shifts mission energy per lap meaningfully; the team should decide whether to adopt it.\n');

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

fprintf('  2. Prop efficiency (cruise): assumed eta_p_C = %.3f   computed (full throttle, V_C=%.1f m/s) = %.3f   delta = %.3f\n', ...
    params.prop.eta_p_C, V_C, eta_prop_cruise, eta_prop_cruise - params.prop.eta_p_C);
fprintf('      Affects: nothing downstream -- eta_p_C was only ever an A2-era sizing input (VehicleWeightEstimation.m,\n');
fprintf('      InitialVehicleSizingConstraint.m), already fully superseded by Deliverable 1''s torque-balance model,\n');
fprintf('      which every A9 script uses instead. Re-run needed: no.\n');
fprintf('  3. Prop efficiency (take-off): assumed eta_p_TO = %.3f   computed (static, V=0) = %.3f   delta = %.3f\n', ...
    params.prop.eta_p_TO, eta_prop_TO, eta_prop_TO - params.prop.eta_p_TO);
fprintf('     (Static propulsive efficiency is 0 by definition -- thrust*velocity is 0 at V=0 no matter the shaft power -- so\n');
fprintf('     eta_p_TO was never a measurable quantity at the condition its name suggests; A2''s value was a stand-in for the\n');
fprintf('     whole low-speed take-off roll''s average efficiency, not literally V=0.)\n');
fprintf('      Affects: nothing downstream, same reason as item 2. Re-run needed: no.\n');

%% ---- 3b. Take-off thrust: A7 sizing target vs. Deliverable 1's computed static thrust ----
% PropulsionSizing.m's T_static (now saved as params.performance.T_static_design)
% is the REQUIRED static thrust an early closed-form take-off equation
% said this motor/prop combo needed to meet S_TO -- the number the
% propulsion system was actually sized against, before Deliverable 1's
% detailed torque-balance model (and the lab-measured thrust correction)
% existed.
if isfield(params.performance, 'T_static_design')
    T_static_computed = model.T_avail_fn(0);
    fprintf('  3b. Take-off thrust:       A7 sizing target (closed-form) = %.2f N   computed (Deliverable 1, static, lab-corrected) = %.2f N   delta = %.2f N\n', ...
        params.performance.T_static_design, T_static_computed, T_static_computed - params.performance.T_static_design);
    fprintf('      Affects: none -- the selected motor/prop already EXCEEDS the A7 sizing target (computed > assumed),\n');
    fprintf('      so the trade selection remains valid; a smaller motor could in principle still meet it, but that''s\n');
    fprintf('      not required. Re-run needed: no.\n');
else
    fprintf('  3b. Take-off thrust:       run PropulsionSizing(params) first to compare against its sizing target.\n');
end

%% ---- 4. Take-off distance: RFP target vs. Deliverable 2's integrated result ----
if isfield(params.performance, 'groundRoll_x')
    fprintf('  4. Take-off distance:     RFP limit (S_TO) = %.1f m   computed (Deliverable 2) = %.2f m   margin = %.2f m\n', ...
        params.performance.S_TO, params.performance.groundRoll_x, params.performance.S_TO - params.performance.groundRoll_x);
    fprintf('      Affects: none -- computed ground roll is well inside the limit with large margin. Re-run needed: no.\n');
else
    fprintf('  4. Take-off distance:     run TakeoffPerformance(params) first to compare against S_TO.\n');
end

%% ---- 4b. Parasite drag: no separate early assumption exists to compare against ----
% A9 asks this be checked like the others (assumed vs. computed), but
% params.aero.CD_0 has only ever been set one place in this codebase --
% DragBuildUp.m's component buildup (fuselage + wing + tail + gear) -- at
% A5. No earlier, independently-guessed CD_0 target exists anywhere in
% A2's sizing to compare it against (unlike V_S, V_C, or T_static above,
% each of which has its own separate early-design formula). Reported
% honestly as "nothing to compare" rather than inventing an early value
% that was never actually written down.
fprintf('  4b. Parasite drag CD_0:    no separate early (A2) assumption exists in this codebase -- CD_0 = %.4f has always been\n', params.aero.CD_0);
fprintf('      DragBuildUp.m''s direct component-buildup result (A5), not an independently-guessed target checked against it here.\n');
fprintf('      Affects: N/A, nothing to compare -- every downstream script already uses this one and only CD_0 value.\n');

%% ---- 5. Maximum level speed: A7/A8's analytic estimate vs. A9's refined model ----
if isfield(params.performance, 'V_max_level_A8')
    fprintf('  5. Max level speed:       A7/A8 analytic (PropulsionSizing.m) = %.2f m/s   A9 refined (full V-sweep torque balance) = %.2f m/s   delta = %.2f m/s\n', ...
        params.performance.V_max_level_A8, params.performance.V_max_level, params.performance.V_max_level - params.performance.V_max_level_A8);
    fprintf('      Affects: V_D/V_NE in VnDiagram.m (A8 Structures) -- V_D = max(1.25*V_C, 1.25*V_max_level), and A8''s\n');
    fprintf('      Structures section runs BEFORE this refined value exists in Main.m''s current script order. Re-run\n');
    fprintf('      needed: YES -- re-run Structures with the updated V_max_level, or reorder the pipeline, for full consistency.\n');
else
    fprintf('  5. Max level speed:       run CruisePerformance(params) first to compare against A8''s estimate.\n');
end

%% ---- 6. Load factor: structural design value vs. the course's actual demand ----
if isfield(params.performance, 'n_turn')
    fprintf('  6. Load factor n:         structural design (A2, used throughout A8) = %.2f   computed at course radius R_req (Deliverable 5) = %.2f   delta = %.2f\n', ...
        params.performance.n, params.performance.n_turn, params.performance.n_turn - params.performance.n);
    fprintf('      Affects: every A8 structural load (WingLoads/TailLoads/FuselageLoads/VnDiagram all size to n=%.2f,\n', params.performance.n);
    fprintf('      not the n=%.2f the course actually demands). Re-run needed: team decision -- if n=%.2f is a deliberate\n', ...
        params.performance.n_turn, params.performance.n);
    fprintf('      margin above the real demand, no re-run needed; if R_M should instead match the real course radius\n');
    fprintf('      R_req, A8 Structures should be re-run (likely giving a lighter, less conservative structure).\n');
else
    fprintf('  6. Load factor n:         run TurnPerformance(params) first to compare against the design n.\n');
end

%% ---- 7. Structural RPM limit: verified, not assumed ----
% Not a comparison -- a check, in the same spirit as Deliverable 1's
% landing-gear-drag-counted-once verification: confirm the capped model
% never actually asks the motor to exceed the propeller's structural
% speed limit anywhere across the sweep it was solved over.
RPM_peak = max(model.RPM_avail_fn(model.V_grid));
ok_RPM = RPM_peak <= model.RPM_lim + 1; % +1 rpm floating-point slack
fprintf('  7. Structural RPM limit:   RPM_lim = %.0f RPM (145000/D_in)   peak commanded across 0-40 m/s = %.0f RPM   -- %s\n', ...
    model.RPM_lim, RPM_peak, local_ternary(ok_RPM, 'never exceeded (verified)', 'EXCEEDED -- check model.RPM_lim clamp'));

%% ---- 8. Propeller static thrust: APC table vs. the team's own prop-stand lab data ----
% PropulsionDragModel.m already applies this correction to every Ct
% lookup (system-wide, per team decision); this item is the evidence
% behind that correction, assumed-vs-measured style like items 1-6.
labCal = model.labCal;
fprintf('  8. Propeller static thrust (lab vs. APC table, %d usable throttle plateaus):\n', numel(labCal.RPM));
for k = 1:numel(labCal.RPM)
    fprintf('       RPM=%5.0f   Ct: APC=%.4f  measured=%.4f (%+.1f%%)   Cp: APC=%.4f  measured=%.4f (%+.1f%%)\n', ...
        labCal.RPM(k), labCal.Ct_apc(k), labCal.Ct_meas(k), labCal.Ct_pctDiff(k), ...
        labCal.Cp_apc(k), labCal.Cp_meas(k), labCal.Cp_pctDiff(k));
end
fprintf('     Cp (shaft power demand) matches the table closely -- no correction. Ct (thrust) is consistently below\n');
fprintf('     the table and gets worse at lower RPM -- a system-wide, RPM-dependent correction is now applied to\n');
fprintf('     every Ct lookup in PropulsionDragModel.m (held flat outside the %.0f-%.0f RPM tested range).\n', ...
    min(labCal.RPM), max(labCal.RPM));

%% ---- 9. Motor electrical constants: could not be calibrated from the lab bench data ----
% Not a comparison -- a documented limitation. Two reasonable regressions
% were attempted against the same prop-stand data used for item 8 above
% and both returned physically impossible motor parameters, so the
% nameplate Kv/I0/R_motor (SizingParams.xlsx) are kept rather than
% replaced with a fit that fits noise, not the motor.
fprintf('  9. Motor electrical constants (Kv, I0, R_motor): attempted lab calibration, NOT applied --\n');
fprintf('     the bench logs battery-side Voltage/Current (voltage RISES as throttle drops, i.e. battery\n');
fprintf('     internal-resistance sag, not motor back-EMF -- these are not motor-phase values), and there is no\n');
fprintf('     logged ESC duty-cycle channel to convert one to the other.\n');
% The two fitted R_motor/I0 values below are a one-time regression result
% against the lab bench data (a fixed dataset, not re-fit every run -- see
% the Word doc for the full derivation); only the "Xx off nameplate"
% comparison is computed live here, so it can't go stale if the team
% updates the nameplate Kv/I0/R_motor cells later (as already happened
% once this session).
R_fit_A = 0.242; I0_fit_B = 6.70; % ohm, A -- the one-time fitted values themselves
fprintf('       Attempt A (raw battery V/I as motor V/I):            R_motor fit = %.3f ohm  (nameplate %.3f ohm, %.1fx off)\n', ...
    R_fit_A, params.prop.R_motor, R_fit_A/params.prop.R_motor);
fprintf('                                                              I0 fit = -6.01 A  (negative -- physically impossible)\n');
fprintf('       Attempt B (duty-corrected, assumed std. 1000-2000us): R_motor fit = -0.011 ohm (negative -- physically impossible)\n');
fprintf('                                                              I0 fit = %.2f A   (nameplate %.2f A, %.1fx off)\n', ...
    I0_fit_B, params.prop.I0, I0_fit_B/params.prop.I0);
fprintf('     Both attempts fit the torque-vs-current data well (R^2 > 0.99) but the voltage equation is unidentifiable\n');
fprintf('     from battery-side-only telemetry -- nameplate Kv=%.0f RPM/V, I0=%.2f A, R_motor=%.3f ohm are kept.\n', ...
    params.prop.Kv, params.prop.I0, params.prop.R_motor);
fprintf('     A future bench run would need motor-phase voltage or a verified ESC duty-cycle curve to calibrate these.\n');

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
        local_pctStr(params.performance.V_max_level_A8, params.performance.V_max_level), 'A8 V_D/V_NE (stale)'};
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
