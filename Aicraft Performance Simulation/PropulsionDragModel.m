function [outputs, params] = PropulsionDragModel(params)

%% A9 Deliverable 1 -- The two models
%
% States and builds the drag model and the propulsion model that every
% other A9 script (TakeoffPerformance, ClimbPerformance, CruisePerformance,
% TurnPerformance) calls into, so the whole assignment shares one
% consistent set of curves rather than five independent reimplementations.
%
% Drag model: the TRIMMED drag polar (CD as a function of CL, accounting
% for the tail lift needed to trim the pitching moment), re-derived here
% from the same formulas TrimAircraft.m already uses, but exposed as a
% reusable function handle (TrimAircraft.m only plots it; it never saves
% CD_trim back into params).
%
% Propulsion model: the same APC-propeller-table + motor-torque-balance
% model PropulsionSizing.m already uses (Ct/Cp vs advance ratio from the
% real prop data file, a motor model from Kv/I0/R_motor -- no fixed
% propulsive efficiency anywhere), generalized here to solve the
% equilibrium at EVERY (throttle, airspeed) pair instead of only at
% V=0 -- PropulsionSizing.m's existing sweep assumes RPM is set by
% throttle alone and holds constant with airspeed, which is a reasonable
% simplification for A7/A8's purposes but not fine-grained enough for A9's
% take-off/climb/cruise/turn integration, which all need thrust and
% electrical power at many different airspeeds.

%% ---- Mass source: the MassBudget sheet, not the iterated sizing result ----
% A8's structural scripts (WingLoads.m, TailLoads.m, FuselageLoads.m,
% TailDraggerTakeOff.m, VnDiagram.m, VnOperatingEnvelope.m) all use a
% separate 'MassBudget' sheet in SizingParams.xlsx -- a component mass-
% station table -- as the source of truth for aircraft weight, by
% deliberate team decision, rather than params.performance.MTOM/MTOW
% (the iterated result of VehicleWeightEstimation.m's weight-fraction
% sizing loop). A9 now follows the same convention, per the team's
% direction: overridden here, the first A9 script every other A9
% deliverable (D2-D9) calls into, so the corrected mass propagates
% through params.performance.MTOM/MTOW automatically -- no need to edit
% each of the other 8 scripts individually, since they already read
% those same fields. The iterated value is left untouched everywhere
% else in the pipeline (A5B, Structures, etc. already ran before this
% point); only A9's own copy of params is affected from here on.
massTable = readtable('SizingParams.xlsx', 'Sheet', 'MassBudget');
MTOM_massTable = sum(massTable.Mass_g)/1000; % kg
MTOM_iterated = params.performance.MTOM;
pct_diff = 100*(MTOM_massTable - MTOM_iterated)/MTOM_iterated;
fprintf('\n--- A9 mass source (Deliverable 1) ---\n');
fprintf('  MassBudget sheet (source of truth, same as A8): %.4f kg\n', MTOM_massTable);
fprintf('  Iterated sizing result (VehicleWeightEstimation.m, no longer used by A9): %.4f kg (%+.1f%% different)\n', ...
    MTOM_iterated, pct_diff);
params.performance.MTOM = MTOM_massTable;
params.performance.MTOW = MTOM_massTable * params.env.g;

rho = params.env.rho;
IN2M = 0.0254;
MS2MPH = 2.23694;

%% ---- Drag model: trimmed polar, CD(CL) ----
% Same derivation as Stability/TrimAircraft.m's "Trim Drag" section,
% generalized to a function handle so the rest of A9 can query it at any
% CL, not just the fixed sample points TrimAircraft.m happens to plot.
CD_trim_fn = @(CL) local_CD_trim(CL, params);

CL_check = linspace(0, params.aero.CL_max, 50);
CD_trim_check = CD_trim_fn(CL_check);
CD_clean_check = params.aero.CD_0 + params.aero.K_wing*CL_check.^2;

% A9 explicitly warns the closed-form best-range/endurance expressions
% assume a parabolic polar centred at CL=0 (CD = CD_0 + K*CL^2) -- check
% that here rather than assuming it. The trim term shifts the CL_wing/
% CL_hstab split linearly with CL, so CD_trim is still quadratic in CL
% but is NOT centred at CL=0 in general; confirm by comparing to the best
% quadratic fit.
pfit = polyfit(CL_check, CD_trim_check, 2);
CD_trim_is_parabolic_at_zero = abs(pfit(2)) < 0.02*abs(pfit(1)*1); % linear-term coefficient small vs quadratic
CL_minDrag = -pfit(2)/(2*pfit(1)); % vertex of the fitted parabola

%% ---- Propulsion model: prop table + motor torque balance ----
Kt = 60 / (2*pi*params.prop.Kv); % [N*m/A]
prop = loadPropData(params.prop.prop_file);
D_prop = IN2M * str2double(regexp(params.prop.prop_file, '_(\d+)x', 'tokens', 'once'));
R_prop = D_prop/2;

RPM_lim = 145000 / (D_prop/IN2M); % propeller structural speed limit (same formula as PropulsionSizing.m)

throttle_grid = (0.10:0.05:1.00)'; % commanded throttle fraction of battery voltage; below ~10% the motor can't reliably overcome I0, so the grid starts there
V_grid = linspace(0, 40, 81); % m/s, same upper bound as PropulsionSizing.m's own sweep

nT = numel(throttle_grid);
nV = numel(V_grid);
RPM_2D   = nan(nT, nV);
T_2D     = nan(nT, nV);
Pelec_2D = nan(nT, nV);
I_2D     = nan(nT, nV);

for i = 1:nT
    V_eff = throttle_grid(i) * params.prop.V_batt;
    RPM_noload = V_eff * params.prop.Kv; % this throttle's no-load speed -- the natural upper bracket bound
    RPM_bracket_hi = min(RPM_noload, 24500); % the APC tables run to 25000 RPM (loadPropData.m); stay inside them
    for j = 1:nV
        V_mph = MS2MPH * V_grid(j);
        Qres = @(RPM) max(Kt*((V_eff - Kt*(2*pi/60)*RPM)/params.prop.R_motor - params.prop.I0), 0) ...
                     - prop.query(RPM, V_mph).Cp * rho * (RPM/60)^2 * D_prop^5 / (2*pi);

        % The tables' RPM=1000 minimum (same lower bound PropulsionSizing.m
        % uses, but only ever at V=0) stops being a valid bracket endpoint
        % as airspeed increases: low RPM at high V is an unrealistically
        % high advance ratio, outside what APC tabulated, so Qres(1000)
        % itself can come back NaN there. Scan up for the lowest RPM that's
        % still finite before calling fzero, rather than assuming 1000
        % always works.
        RPM_lo = NaN;
        for rc = 1000:500:RPM_bracket_hi
            if isfinite(Qres(rc)), RPM_lo = rc; break, end
        end
        if isnan(RPM_lo), continue, end % no in-range RPM anywhere in the bracket at this (throttle,V) -- genuinely no result

        try
            RPM_eq = fzero(Qres, [RPM_lo, RPM_bracket_hi]);
        catch
            continue % no equilibrium in range, or the (RPM,V) point falls outside the tabulated prop data -- report no result (NaN), per A9 D1's instruction, rather than clamping
        end
        RPM_final = min(RPM_eq, RPM_lim);
        r = prop.query(RPM_final, V_mph);
        if ~r.inRange, continue, end % outside the tabulated envelope at the capped RPM too -- still no result
        n_rps = RPM_final/60; omega = 2*pi*n_rps;
        T = r.Ct * rho * n_rps^2 * D_prop^4;
        P_shaft = r.Cp * rho * n_rps^3 * D_prop^5;
        I = P_shaft/(Kt*omega) + params.prop.I0;
        P_elec = (omega*Kt*I + I^2*params.prop.R_motor) / params.prop.eta_ESC;

        RPM_2D(i,j)   = RPM_final;
        T_2D(i,j)     = T;
        Pelec_2D(i,j) = P_elec;
        I_2D(i,j)     = I;
    end
end

%% ---- Throttle cap: RFP power limit, motor power rating, motor current limit ----
% ASSUMPTION -- see the Word document: P_limit_RFP, P_motor_rated and
% I_motor_limit are placeholders (SizingParams.xlsx, added for A9) until
% the team confirms the real RFP power-limit rule and the motor's
% datasheet ratings.
P_cap = min(params.performance.P_limit_RFP, params.prop.P_motor_rated);
I_cap = params.prop.I_motor_limit;

ok_2D = (Pelec_2D <= P_cap) & (I_2D <= I_cap) & ~isnan(T_2D);

throttle_cap   = nan(1, nV);
T_avail        = nan(1, nV);
Pelec_avail    = nan(1, nV);
I_avail        = nan(1, nV);
RPM_avail      = nan(1, nV);
throttle_bound = false(1, nV); % true where the cap actually reduces throttle below 100%

for j = 1:nV
    idx_ok = find(ok_2D(:,j));
    if isempty(idx_ok), continue, end
    [~, k] = max(throttle_grid(idx_ok)); % the largest throttle at this V that still satisfies every limit
    i_best = idx_ok(k);
    tau_cap = throttle_grid(i_best);
    T_cap = T_2D(i_best,j); Pelec_cap = Pelec_2D(i_best,j); I_cap_j = I_2D(i_best,j); RPM_cap = RPM_2D(i_best,j);

    % Snapping to the nearest throttle-grid point otherwise makes T_avail
    % (and anything built from it, e.g. rate of climb) step discontinuously
    % as V crosses whichever airspeed moves the binding grid point -- if
    % the NEXT grid throttle up is invalid, linearly interpolate the exact
    % throttle where the binding limit (power or current) is crossed,
    % between these two already-computed grid points, for a smooth curve.
    if i_best < nT && ~ok_2D(i_best+1,j) && ~isnan(Pelec_2D(i_best+1,j)) && ~isnan(I_2D(i_best+1,j))
        frac_P = Inf; frac_I = Inf;
        if Pelec_2D(i_best+1,j) > P_cap
            frac_P = (P_cap - Pelec_2D(i_best,j)) / (Pelec_2D(i_best+1,j) - Pelec_2D(i_best,j));
        end
        if I_2D(i_best+1,j) > I_cap
            frac_I = (I_cap - I_2D(i_best,j)) / (I_2D(i_best+1,j) - I_2D(i_best,j));
        end
        frac = min(frac_P, frac_I);
        if isfinite(frac) && frac >= 0 && frac <= 1
            tau_cap   = throttle_grid(i_best)   + frac*(throttle_grid(i_best+1)   - throttle_grid(i_best));
            T_cap     = T_2D(i_best,j)          + frac*(T_2D(i_best+1,j)          - T_2D(i_best,j));
            Pelec_cap = Pelec_2D(i_best,j)      + frac*(Pelec_2D(i_best+1,j)      - Pelec_2D(i_best,j));
            I_cap_j   = I_2D(i_best,j)          + frac*(I_2D(i_best+1,j)          - I_2D(i_best,j));
            RPM_cap   = RPM_2D(i_best,j)        + frac*(RPM_2D(i_best+1,j)        - RPM_2D(i_best,j));
        end
    end

    throttle_cap(j) = tau_cap;
    T_avail(j)      = T_cap;
    Pelec_avail(j)  = Pelec_cap;
    I_avail(j)      = I_cap_j;
    RPM_avail(j)    = RPM_cap;
    throttle_bound(j) = throttle_cap(j) < 0.999;
end

% An isolated V where the torque-balance fzero solve happened to fail
% (bracket edge case, not a genuine physical gap) would otherwise poison
% every downstream interpolation query near it. Fill INTERIOR gaps only
% (linear across their valid neighbours) -- a leading/trailing run of NaN,
% i.e. a genuine "no throttle reaches this airspeed" region, is left as
% NaN, preserving the "report no result" convention at the true edges.
throttle_cap = local_fillInteriorNaN(throttle_cap);
T_avail      = local_fillInteriorNaN(T_avail);
Pelec_avail  = local_fillInteriorNaN(Pelec_avail);
I_avail      = local_fillInteriorNaN(I_avail);
RPM_avail    = local_fillInteriorNaN(RPM_avail);

T_avail_fn     = griddedInterpolant(V_grid, T_avail, 'linear', 'nearest');
Pelec_avail_fn = griddedInterpolant(V_grid, Pelec_avail, 'linear', 'nearest');
I_avail_fn     = griddedInterpolant(V_grid, I_avail, 'linear', 'nearest');
throttle_cap_fn = griddedInterpolant(V_grid, throttle_cap, 'linear', 'nearest');
RPM_avail_fn   = griddedInterpolant(V_grid, RPM_avail, 'linear', 'nearest');

% Full 2-D grids too (throttle AND airspeed), for CruisePerformance.m's
% and TurnPerformance.m's "what throttle gives level flight at this V"
% search. Fill along the V direction only (dimension 2): a gap there is
% the same isolated-solver-hiccup case as above. A gap along the
% throttle direction (dimension 1, fixed V) is NOT filled -- at a fixed
% V, low throttle can genuinely have no valid equilibrium (too low an
% RPM for that airspeed, outside the tabulated envelope) while higher
% throttle does; that is a real feature of the data, not a numerical
% artifact, so find_throttle_for_thrust below scans for a valid bracket
% instead of assuming the lowest throttle grid point is usable.
T_2D_fill     = fillmissing(T_2D, 'linear', 2, 'EndValues', 'none');
Pelec_2D_fill = fillmissing(Pelec_2D, 'linear', 2, 'EndValues', 'none');
I_2D_fill     = fillmissing(I_2D, 'linear', 2, 'EndValues', 'none');
T_grid_fn     = griddedInterpolant({throttle_grid, V_grid}, T_2D_fill, 'linear', 'none');
Pelec_grid_fn = griddedInterpolant({throttle_grid, V_grid}, Pelec_2D_fill, 'linear', 'none');
I_grid_fn     = griddedInterpolant({throttle_grid, V_grid}, I_2D_fill, 'linear', 'none');
find_throttle_for_thrust = @(V, T_req) local_findThrottleForThrust(V, T_req, throttle_grid, T_grid_fn);

%% ---- Landing-gear drag counted once: verify, don't assume ----
% DragBuildUp.m sums CD_0_gear into CD_0_comp exactly once (fuselage +
% wing + hstab + vstab + gear), and nothing downstream of it (calcAero.m's
% CD_TO/CD_G/CD_C, PropulsionSizing.m's D_level, TailDraggerTakeOff.m's
% D_fn) re-adds a gear term -- all of them just reuse params.aero.CD_0
% as a single number. Report the gear fraction as evidence, not assertion.
gear_fraction_of_CD0 = NaN;
if isfield(params.aero, 'CD_0') && params.aero.CD_0 > 0
    % CD_0_gear itself isn't saved by DragBuildUp.m; recompute it the same way here just to report the fraction.
    A_main_wheel = params.geometry.main_wheel_dia * params.geometry.main_wheel_width;
    A_main_strut = params.geometry.main_gear_len * params.geometry.main_strut_dia;
    A_tail_wheel = params.geometry.tail_wheel_dia * params.geometry.tail_wheel_width;
    A_tail_strut = params.geometry.tail_gear_len * params.geometry.tail_strut_dia;
    fe_gear = 2*params.aero.DqA_wheel*A_main_wheel + 2*params.aero.DqA_strut*A_main_strut ...
            + params.aero.DqA_wheel*A_tail_wheel + params.aero.DqA_strut*A_tail_strut;
    CD_0_gear = fe_gear / params.geometry.S_wing;
    gear_fraction_of_CD0 = CD_0_gear / params.aero.CD_0;
end

%% ---- Console summary ----
fprintf('\n--- Deliverable 1: Propulsion & Drag Models (A9) ---\n');
fprintf('  \nDrag model: trimmed polar CD(CL) = CD_0 + K_wing*CL_wing^2 + (St/S)*K_hstab*CL_hstab^2\n');
fprintf('    (clean polar at CL_C=%.2f: CD=%.4f;  trimmed: CD=%.4f)\n', ...
    params.aero.CL_C, params.aero.CD_0 + params.aero.K_wing*params.aero.CL_C^2, CD_trim_fn(params.aero.CL_C));
if ~CD_trim_is_parabolic_at_zero
    fprintf('  \n>>> FINDING: the trimmed polar is NOT centred at CL=0 (min-drag CL = %.3f) -- the lecture''s closed-form\n', CL_minDrag);
    fprintf('      best-range/endurance formulas do not apply as-is; CruisePerformance.m searches numerically instead.\n');
end
fprintf('  \nPropulsion model: APC prop table (%s) + motor torque balance (Kv=%.0f, I0=%.2f A, R=%.3f ohm) -- no fixed eta_prop.\n', ...
    params.prop.prop_file, params.prop.Kv, params.prop.I0, params.prop.R_motor);
fprintf('  \nMaximum shaft speed: structural limit 145000/D_in = %.0f RPM (D = %.1f in); torque-balance equilibrium is capped there when it would exceed it.\n', ...
    RPM_lim, D_prop/IN2M);
fprintf('  \nLanding-gear drag: CD_0_gear / CD_0 = %.1f%% of parasite drag, summed once into CD_0_comp (DragBuildUp.m) and never re-added downstream.\n', 100*gear_fraction_of_CD0);
fprintf('  \nUsable battery energy fraction used throughout: useableCapacity=%.0f%% x temp_derate=%.0f%% = %.0f%% of the nominal pack.\n', ...
    100*params.prop.useableCapacity, 100*params.prop.temp_derate, 100*params.prop.useableCapacity*params.prop.temp_derate);
fprintf('  \nThrottle cap: P_limit_RFP=%.0f W, P_motor_rated=%.0f W, I_motor_limit=%.0f A (ASSUMPTIONS -- see Word doc).\n', ...
    params.performance.P_limit_RFP, params.prop.P_motor_rated, params.prop.I_motor_limit);
if any(throttle_bound)
    V_bind = V_grid(find(throttle_bound, 1));
    fprintf('  >>> FINDING: the cap binds (throttle < 100%%) above %.1f m/s -- full throttle is not achievable there.\n', V_bind);
else
    fprintf('  Cap never binds across the 0-40 m/s sweep -- full throttle stays within all three limits.\n');
end

%% ---- Outputs ----
model = struct( ...
    'CD_trim_fn', CD_trim_fn, 'CL_minDrag', CL_minDrag, 'CD_trim_is_parabolic_at_zero', CD_trim_is_parabolic_at_zero, ...
    'D_prop', D_prop, 'R_prop', R_prop, 'RPM_lim', RPM_lim, ...
    'V_grid', V_grid, 'throttle_grid', throttle_grid, ...
    'T_avail_fn', T_avail_fn, 'Pelec_avail_fn', Pelec_avail_fn, 'I_avail_fn', I_avail_fn, 'throttle_cap_fn', throttle_cap_fn, 'RPM_avail_fn', RPM_avail_fn, ...
    'T_grid_fn', T_grid_fn, 'Pelec_grid_fn', Pelec_grid_fn, 'I_grid_fn', I_grid_fn, 'find_throttle_for_thrust', find_throttle_for_thrust, ...
    'P_cap', P_cap, 'I_cap', I_cap, 'throttle_bound', throttle_bound);

params.prop.model = model; % the reusable propulsion model every other A9 script queries

outputs.PropulsionDragModel = model;

end

function CD = local_CD_trim(CL, params)
    CL_hstab = (params.aero.CM_ac_w + CL*(params.geometry.x_cg_design - params.geometry.x_ac)) / params.geometry.V_H;
    CL_wing  = CL - params.geometry.St_S * CL_hstab;
    CD = params.aero.CD_0 + params.aero.K_wing*CL_wing.^2 + params.geometry.St_S*params.aero.K_hstab*CL_hstab.^2;
end

function tau = local_findThrottleForThrust(V, T_req, throttle_grid, T_grid_fn)
    % Find the throttle giving T_grid_fn(tau,V) = T_req by scanning for
    % an ADJACENT pair of throttle-grid points that are both finite and
    % bracket a sign change, then refining with fzero -- robust to the
    % low-throttle-at-this-V gaps described above the caller. Returns NaN
    % if no such bracket exists (genuinely no result: T_req is not
    % achievable by any throttle at this V).
    tau = NaN;
    f = @(t) T_grid_fn(t, V) - T_req;
    fvals = arrayfun(f, throttle_grid);
    for k = 1:numel(throttle_grid)-1
        f1 = fvals(k); f2 = fvals(k+1);
        if isnan(f1) || isnan(f2), continue, end
        if f1 == 0, tau = throttle_grid(k); return, end
        if f2 == 0, tau = throttle_grid(k+1); return, end
        if sign(f1) ~= sign(f2)
            try
                tau = fzero(f, [throttle_grid(k), throttle_grid(k+1)]);
                return
            catch
            end
        end
    end
end

function y = local_fillInteriorNaN(y)
    % Linearly fill any NaN run that has a valid value on BOTH sides;
    % leading/trailing NaN runs (no valid value on one side) are left
    % untouched, since those represent a genuine "no result" edge, not a
    % numerical gap.
    n = numel(y);
    isn = isnan(y);
    if ~any(isn), return, end
    i = 1;
    while i <= n
        if isn(i)
            j = i;
            while j <= n && isn(j), j = j+1; end
            if i > 1 && j <= n % bounded on both sides -- interior gap
                y(i:j-1) = interp1([i-1, j], [y(i-1), y(j)], i:j-1);
            end
            i = j;
        else
            i = i+1;
        end
    end
end
