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
fprintf('  3. Prop efficiency (take-off): assumed eta_p_TO = %.3f   computed (static, V=0) = %.3f   delta = %.3f\n', ...
    params.prop.eta_p_TO, eta_prop_TO, eta_prop_TO - params.prop.eta_p_TO);
fprintf('     (Static propulsive efficiency is 0 by definition -- thrust*velocity is 0 at V=0 no matter the shaft power -- so\n');
fprintf('     eta_p_TO was never a measurable quantity at the condition its name suggests; A2''s value was a stand-in for the\n');
fprintf('     whole low-speed take-off roll''s average efficiency, not literally V=0.)\n');

%% ---- 4. Take-off distance: RFP target vs. Deliverable 2's integrated result ----
if isfield(params.performance, 'groundRoll_x')
    fprintf('  4. Take-off distance:     RFP limit (S_TO) = %.1f m   computed (Deliverable 2) = %.2f m   margin = %.2f m\n', ...
        params.performance.S_TO, params.performance.groundRoll_x, params.performance.S_TO - params.performance.groundRoll_x);
else
    fprintf('  4. Take-off distance:     run TakeoffPerformance(params) first to compare against S_TO.\n');
end

%% ---- 5. Maximum level speed: A7/A8's analytic estimate vs. A9's refined model ----
if isfield(params.performance, 'V_max_level_A8')
    fprintf('  5. Max level speed:       A7/A8 analytic (PropulsionSizing.m) = %.2f m/s   A9 refined (full V-sweep torque balance) = %.2f m/s   delta = %.2f m/s\n', ...
        params.performance.V_max_level_A8, params.performance.V_max_level, params.performance.V_max_level - params.performance.V_max_level_A8);
else
    fprintf('  5. Max level speed:       run CruisePerformance(params) first to compare against A8''s estimate.\n');
end

%% ---- 6. Load factor: structural design value vs. the course's actual demand ----
if isfield(params.performance, 'n_turn')
    fprintf('  6. Load factor n:         structural design (A2, used throughout A8) = %.2f   computed at course radius R_req (Deliverable 5) = %.2f   delta = %.2f\n', ...
        params.performance.n, params.performance.n_turn, params.performance.n_turn - params.performance.n);
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
fprintf('       Attempt A (raw battery V/I as motor V/I):            R_motor fit = 0.242 ohm  (nameplate 0.029 ohm, 8x off)\n');
fprintf('                                                              I0 fit = -6.01 A  (negative -- physically impossible)\n');
fprintf('       Attempt B (duty-corrected, assumed std. 1000-2000us): R_motor fit = -0.011 ohm (negative -- physically impossible)\n');
fprintf('                                                              I0 fit = 6.70 A   (nameplate 1.39 A, 5x off)\n');
fprintf('     Both attempts fit the torque-vs-current data well (R^2 > 0.99) but the voltage equation is unidentifiable\n');
fprintf('     from battery-side-only telemetry -- nameplate Kv=%.0f RPM/V, I0=%.2f A, R_motor=%.3f ohm are kept.\n', ...
    params.prop.Kv, params.prop.I0, params.prop.R_motor);
fprintf('     A future bench run would need motor-phase voltage or a verified ESC duty-cycle curve to calibrate these.\n');

%% ---- Outputs ----
outputs.AssumptionValidation = struct('V_S_assumed', V_S_assumed, 'V_S_computed', V_S_computed, ...
    'eta_p_C_assumed', params.prop.eta_p_C, 'eta_p_C_computed', eta_prop_cruise, ...
    'eta_p_TO_assumed', params.prop.eta_p_TO, 'eta_p_TO_computed', eta_prop_TO, ...
    'RPM_peak', RPM_peak, 'RPM_lim', model.RPM_lim, 'ok_RPM', ok_RPM, ...
    'propLabCal', labCal, 'motorCal_applied', false);

end

function out = local_ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
