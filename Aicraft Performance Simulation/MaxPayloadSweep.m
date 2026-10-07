function [outputs, params] = MaxPayloadSweep(params)

%% A9 Deliverable 8 -- Maximum Payload
%
% A9's own words: "Assume the fuselage has room for the payload and that
% it is placed so the CG does not move. Keep the airframe, battery, motor
% and propeller as designed." -- i.e. only the payload mass varies; the
% rest of the aircraft (wing area, drag polar, CG location, battery
% capacity, the propulsion model) is FIXED at the baseline design. An
% earlier version of this script instead re-ran the full weight-
% convergence loop (InitialCalcs/DragBuildUp/calcAero/
% VehicleWeightEstimation) at every candidate, which resizes the BATTERY
% along with payload (VehicleWeightEstimation.m sizes it as a fixed
% weight fraction) -- directly contradicting "keep the battery as
% designed," and the reason an earlier run's laps-possible curve rose
% before falling instead of simply falling (a growing battery was
% masking the real trend) and why the payload range topped out
% implausibly low. Fixed: MTOM now grows exactly 1:1 with added payload
% on top of a FIXED empty+battery mass, and only the weight-DEPENDENT
% performance scripts (TakeoffPerformance, ClimbPerformance, the turn
% calculation, MissionSimulation) are re-run per candidate -- not A5B or
% PropulsionDragModel, since nothing about the airframe/propulsion design
% itself changes. This is also ~10-20x faster per candidate, so the sweep
% now runs well past the RFP's MTOW cap for context (per the team's
% request) instead of stopping exactly at it.
%
% Six things are tracked against their own limit (eight individual
% checks once the turn and peak-draw items are split into their two
% parts each):
%   1. Ground roll, against the RFP limit (R1, 25 m).
%   2. Stall speed, against the landing requirement (LANDING_SPEED_RATIO
%      x the baseline self-consistent stall speed -- an invented
%      placeholder ratio; see item below). Where this payload is reached
%      is solved in closed form (V_S is a plain algebraic function of
%      mass), not interpolated off the swept grid like the other seven.
%   3. Maximum rate of climb (and time to the RFP altitude).
%   4. The course turn: lift coefficient against CL_max WITH the stall
%      margin already used elsewhere in this codebase (CL_R =
%      CL_max/mult_V_TO^2, the same margin InitialCalcs.m already applies
%      at rotation) -- not raw CL_max -- and power required against power
%      available at that turn.
%   5. Laps possible, against the 3 required (R3/N2).
%   6. Peak electrical power against the RFP limit (R21, 1000 W) AND peak
%      current against the motor/controller rating (SizingParams.xlsx
%      I_motor_limit).
% The maximum payload actually reported is the lower of (a) whichever of
% these eight fails at the lowest payload, and (b) the payload at which
% MTOW itself reaches the RFP's 6 kg cap (R22) -- MTOW is not one of the
% six listed quantities, but it is still a hard requirement (R22) that
% the aircraft cannot legally exceed even if every other quantity still
% has margin. Both this binding payload AND the MTOW cap payload are
% marked on every panel (separate vertical lines), and the sweep itself
% continues past both so each quantity's own natural trend is visible.
%
% ASSUMPTION (LANDING_SPEED_RATIO below): no numeric landing speed
% requirement exists anywhere in this project's prior assignments or the
% team's requirements document (R5 only says "lands on grass without
% damage," no speed). Per the team's direction, a placeholder ratio of
% 1.25x the baseline self-consistent stall speed stands in (1.1-1.3x
% V_stall is typical practice for this class of RC aircraft; full-scale
% aircraft use 1.3x, RC/DBF designs commonly land a bit slower since
% there's no passenger comfort to protect). Computed fresh from V_S here
% -- not read from a fixed SizingParams.xlsx cell -- so it automatically
% tracks V_S if the design changes; the computed value is still WRITTEN
% back to SizingParams.xlsx (row 109) each run, purely as a visible
% record, not as something this script reads back in.
LANDING_SPEED_RATIO = 1.25;

if ~isfield(params.performance, 'W_pay')
    error('MaxPayloadSweep:noBaseline', 'params must already contain a baseline W_pay (run the normal Main.m pipeline first).');
end
if ~isfield(params.prop, 'model')
    error('MaxPayloadSweep:noModel', 'Run PropulsionDragModel(params) first -- this script reuses that FIXED model for every candidate, it does not re-solve it.');
end

figs_keep = findall(0, 'Type', 'figure');
warnState = warning('off', 'all');
cleanupWarn = onCleanup(@() warning(warnState)); %#ok<NASGU>

%% ---- Fixed aircraft: baseline empty+battery mass, payload grows on top ----
MTOW_CAP_KG = 6.0; % R22
W_pay_baseline = params.performance.W_pay;
MTOM_fixed = params.performance.MTOM - W_pay_baseline; % empty structure + battery, held constant per A9's instruction
W_pay_at_MTOWcap = MTOW_CAP_KG - MTOM_fixed; % trivial now: MTOM grows 1:1 with payload, no resizing loop needed

if W_pay_at_MTOWcap <= 0
    error('MaxPayloadSweep:overweight', 'Baseline empty+battery mass (%.3f kg) already exceeds the 6 kg MTOW cap with zero payload.', MTOM_fixed);
end

N_PTS = 16;
W_pay_grid = linspace(0.02, 1.6*W_pay_at_MTOWcap, N_PTS); % swept well past the MTOW cap so each quantity's own trend is visible, per the team's request

% Baseline self-consistent stall speed (as-designed aircraft, zero added
% payload) -- same formula AssumptionValidation.m/CruisePerformance.m/
% ClimbPerformance.m/local_runCandidate below all already use. The
% landing-speed limit is a fixed property of the aircraft/pilot (how fast
% it's allowed to touch down), so it's derived once here from the
% baseline design, not recomputed per sweep candidate, and threaded into
% local_runCandidate below rather than read from params there.
rho = params.env.rho; S = params.geometry.S_wing;
V_S_baseline = sqrt(2*params.performance.MTOW/(rho*S*params.aero.CL_max));
V_S_LIMIT = LANDING_SPEED_RATIO * V_S_baseline;

fprintf('\n--- Maximum Payload Sweep (Deliverable 8) ---\n');
fprintf('  Fixed empty+battery mass: %.3f kg (baseline MTOM %.3f kg - baseline payload %.3f kg)\n', MTOM_fixed, params.performance.MTOM, W_pay_baseline);
fprintf('  Payload at MTOW = 6 kg (R22): %.3f kg\n', W_pay_at_MTOWcap);
fprintf('  Landing speed limit: %.2f x baseline stall speed (%.2f m/s) = %.2f m/s (ASSUMPTION -- no real requirement exists yet)\n', ...
    LANDING_SPEED_RATIO, V_S_baseline, V_S_LIMIT);
fprintf('  Sweeping W_pay from %.3f to %.3f kg over %d points.\n', W_pay_grid(1), W_pay_grid(end), N_PTS);

% Report the computed value back to SizingParams.xlsx (row 109) purely as
% a visible record of what this run computed -- this script does not read
% that cell back in; see the ASSUMPTION note above local_runCandidate.
try
    writematrix(V_S_LIMIT, 'SizingParams.xlsx', 'Sheet', 'Sheet1', 'Range', 'C109');
    writematrix(sprintf('COMPUTED each run by MaxPayloadSweep.m = %.2fx the baseline self-consistent stall speed -- not read back in as an input, edit LANDING_SPEED_RATIO in that script instead.', LANDING_SPEED_RATIO), ...
        'SizingParams.xlsx', 'Sheet', 'Sheet1', 'Range', 'F109');
catch ME
    fprintf('  (could not write the computed landing-speed limit back to SizingParams.xlsx: %s)\n', ME.message);
end

runLabel = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm'));
logRows = cell(0, 16);

G = struct('MTOM', nan(1,N_PTS), 'S_TO', nan(1,N_PTS), 'V_S', nan(1,N_PTS), ...
    'ROC_max', nan(1,N_PTS), 't_climb', nan(1,N_PTS), ...
    'CL_turn', nan(1,N_PTS), 'CL_limit', nan(1,N_PTS), ...
    'P_req_turn', nan(1,N_PTS), 'P_avail_turn', nan(1,N_PTS), ...
    'laps_possible', nan(1,N_PTS), 'P_peak', nan(1,N_PTS), 'I_peak', nan(1,N_PTS), ...
    'ok', false(1,N_PTS));

for i = 1:N_PTS
    r = local_runCandidate(params, MTOM_fixed, W_pay_grid(i), figs_keep, V_S_LIMIT);
    fn = fieldnames(G);
    for kf = 1:numel(fn)-1 % all but 'ok'
        G.(fn{kf})(i) = r.(fn{kf});
    end
    G.ok(i) = r.ok;
    logRows(end+1, :) = local_logRow(runLabel, sprintf('Grid %d/%d', i, N_PTS), W_pay_grid(i), r); %#ok<AGROW>
end

local_writeLog(logRows, runLabel);

%% ---- Find where each of the 8 checks first fails, by linear interpolation ----
I_CAP = params.prop.I_motor_limit;
P_CAP_RFP = 1000; % R21
S_TO_LIMIT = params.performance.S_TO;

checks = struct( ...
    'name', {'Ground roll', 'Stall speed', 'Max rate of climb', 'Turn CL (stall margin)', ...
              'Turn power required', 'Laps possible', 'Peak electrical power', 'Peak current'}, ...
    'value', {G.S_TO, G.V_S, G.ROC_max, G.CL_turn, G.P_req_turn - G.P_avail_turn, ...
              G.laps_possible, G.P_peak, G.I_peak}, ...
    'limit', {S_TO_LIMIT, V_S_LIMIT, 0, NaN, 0, 3, P_CAP_RFP, I_CAP}, ...
    'sense', {'max', 'max', 'min', 'max', 'max', 'min', 'max', 'max'}); % 'max': fails when value > limit; 'min': fails when value < limit
for i = 1:N_PTS
    checks(4).value(i) = G.CL_turn(i) - G.CL_limit(i); % recast as a margin: fails when > 0
end
checks(4).limit = 0;

W_pay_cross = nan(1, numel(checks));
for c = 1:numel(checks)
    v = checks(c).value; lim = checks(c).limit;
    % NaN means the candidate's chain errored or the condition was
    % physically infeasible to even evaluate (e.g. the turn needs more
    % thrust than is available at all, so there's no throttle to solve
    % for) -- that's already a failure, not a benign gap, but a plain
    % "v > lim" or "v < lim" comparison against NaN silently evaluates to
    % false in MATLAB and would hide a real failure. Treat NaN as failing
    % explicitly.
    if strcmp(checks(c).sense, 'min')
        failed = (v < lim) | isnan(v);
    else
        failed = (v > lim) | isnan(v);
    end
    i_fail = find(failed, 1, 'first');
    if isempty(i_fail)
        W_pay_cross(c) = Inf; % never fails within the swept range
    elseif i_fail == 1 || isnan(v(i_fail-1)) || isnan(v(i_fail))
        % Can't linearly interpolate across or into a NaN point -- place
        % the crossing conservatively at the first failing grid point.
        W_pay_cross(c) = W_pay_grid(i_fail);
    else
        v0 = v(i_fail-1); v1 = v(i_fail);
        frac = (lim - v0) / (v1 - v0);
        frac = min(max(frac, 0), 1);
        W_pay_cross(c) = W_pay_grid(i_fail-1) + frac*(W_pay_grid(i_fail) - W_pay_grid(i_fail-1));
    end
end

% Stall speed is solved in closed form here, not by interpolating the
% swept grid like the other seven checks -- V_S(W_pay) is a plain
% algebraic function of mass (sqrt(2*(MTOM_fixed+W_pay)*g/(rho*S*CL_max))),
% so inverting it directly for V_S = V_S_LIMIT is exact, matching the
% lecture's own closed-form mp,max formula, rather than an approximation
% from 16 grid points (ground roll and laps possible genuinely need the
% sweep -- they come from integrating/simulating, not a formula).
i_stall = find(strcmp({checks.name}, 'Stall speed'));
W_pay_stall_closedform = (rho*S*params.aero.CL_max*V_S_LIMIT^2)/(2*params.env.g) - MTOM_fixed;
W_pay_cross(i_stall) = max(W_pay_stall_closedform, 0);

[W_pay_max_analyses, i_binding] = min(W_pay_cross);

% The payload actually reported can't exceed the MTOW cap either, even if
% every one of the 6 analyzed quantities still has margin there.
if W_pay_at_MTOWcap <= W_pay_max_analyses
    W_pay_max = W_pay_at_MTOWcap;
    binding_name = 'MTOW cap (R22, 6 kg)';
    mtowIsBinding = true;
else
    W_pay_max = W_pay_max_analyses;
    binding_name = checks(i_binding).name;
    mtowIsBinding = false;
end

% Next-binding constraint: the smallest crossing (among the 6 analyses
% AND the MTOW cap) strictly greater than whichever is binding now.
allCrossings = [W_pay_cross, W_pay_at_MTOWcap];
allNames = [{checks.name}, {'MTOW cap (R22, 6 kg)'}];
higherCrossings = allCrossings(allCrossings > W_pay_max);
if isempty(higherCrossings)
    W_pay_next = NaN; next_name = '';
else
    W_pay_next = min(higherCrossings);
    i_next = find(allCrossings == W_pay_next, 1);
    next_name = allNames{i_next};
end

n_cubes = floor(W_pay_max / 0.52); % R14: 520 g per payload cube

fprintf('  >>> Maximum payload: %.3f kg, set by: %s (%d whole payload cubes, R14 @ 0.52 kg each)\n', ...
    W_pay_max, binding_name, n_cubes);
fprintf('  Margins on the other constraints at W_pay = %.3f kg:\n', W_pay_max);
for c = 1:numel(checks)
    if ~mtowIsBinding && c == i_binding, continue, end
    fprintf('    %-24s binds at %.3f kg%s\n', checks(c).name, W_pay_cross(c), ...
        local_ternary(isinf(W_pay_cross(c)), ' (never, within the swept range)', ''));
end
fprintf('    %-24s binds at %.3f kg\n', 'MTOW cap (R22)', W_pay_at_MTOWcap);
if ~isnan(W_pay_next)
    fprintf('  Next-binding constraint if %s is relieved: %s, at %.3f kg\n', binding_name, next_name, W_pay_next);
else
    fprintf('  No other constraint binds within the swept range.\n');
end

%% ---- Plot: all 8 checks against payload, 2x4 panels ----
blue = [0.00 0.45 0.74]; orange = [0.85 0.33 0.10]; red = [0.80 0.00 0.00]; gray = [0.3 0.3 0.3]; purple = [0.49 0.18 0.56];

% Displayed x-axis ends at the R22 MTOW-cap payload (a tiny margin past
% it so that line doesn't sit flush on the right edge, invisible behind
% the axis spine) -- the sweep itself still computes well past that
% point (W_pay_grid goes wider, per the team's earlier request) so every
% quantity's own crossing is still found accurately even when it falls
% beyond R22; this only trims what's drawn, not what's computed.
xlim_pad = [0, W_pay_at_MTOWcap*1.03];

% No explicit 'Position' here -- Setup.m already sets every figure to
% dock by default, and combining 'WindowStyle','docked' with an explicit
% Position pops the figure out as its own separate window instead of
% joining the rest in the same docked tab group (same conflict fixed
% elsewhere earlier this session).
figure('Name','Maximum Payload Sweep','Color','w','WindowStyle','docked');

local_panel(1, W_pay_grid, G.S_TO, S_TO_LIMIT, 'Ground roll (m)', 'Ground roll vs RFP limit', W_pay_max, W_pay_at_MTOWcap, blue, red, gray, purple, [], xlim_pad, 'Ground roll', 'RFP limit, R1 (25 m)');
local_panel(2, W_pay_grid, G.V_S, V_S_LIMIT, 'Stall speed (m/s)', 'Stall speed vs landing limit*', W_pay_max, W_pay_at_MTOWcap, blue, red, gray, purple, [], xlim_pad, 'Stall speed', 'Landing limit*');
local_panel(3, W_pay_grid, G.ROC_max, 0, 'Max rate of climb (m/s)', 'Max ROC vs 0 (climb feasibility)', W_pay_max, W_pay_at_MTOWcap, blue, red, gray, purple, [], xlim_pad, 'Max ROC', 'Cannot climb (0)');
local_panel(4, W_pay_grid, G.CL_turn, NaN, 'Turn C_L (-)', 'Turn CL vs stall-margined limit', W_pay_max, W_pay_at_MTOWcap, blue, red, gray, purple, G.CL_limit, xlim_pad, 'Turn CL', 'Stall-margined limit (CL_R)');

subplot(2,4,5); hold on; box on; grid on;
plot(W_pay_grid, G.P_avail_turn, '-', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', 'Available');
plot(W_pay_grid, G.P_req_turn, '--', 'Color', blue, 'LineWidth', 1.6, 'DisplayName', 'Required');
xline(W_pay_at_MTOWcap, '-.', 'Color', purple, 'DisplayName', 'MTOW cap, R22');
xline(W_pay_max, ':', 'Color', gray, 'LineWidth', 1.3, 'DisplayName', 'Max payload');
xlabel('Payload mass (kg)'); ylabel('Turn power (W)'); title('Turn power required vs available'); legend('Location','best','FontSize',6);
xlim(xlim_pad);

local_panel(6, W_pay_grid, G.laps_possible, 3, 'Laps possible (-)', 'Laps possible vs 3 required', W_pay_max, W_pay_at_MTOWcap, blue, red, gray, purple, [], xlim_pad, 'Laps possible', 'Laps required, R3 (3)');
local_panel(7, W_pay_grid, G.P_peak, P_CAP_RFP, 'Peak electrical power (W)', 'Peak power vs RFP limit (R21)', W_pay_max, W_pay_at_MTOWcap, blue, red, gray, purple, [], xlim_pad, 'Peak power', 'RFP limit, R21 (1000 W)');
local_panel(8, W_pay_grid, G.I_peak, I_CAP, 'Peak current (A)', 'Peak current vs motor/controller limit', W_pay_max, W_pay_at_MTOWcap, blue, red, gray, purple, [], xlim_pad, 'Peak current', 'Motor/controller limit');

sgtitle(sprintf('Maximum payload %.3f kg (%d cubes), set by: %s', W_pay_max, n_cubes, binding_name), ...
    'FontWeight', 'bold', 'FontSize', 13);
annotation('textbox', [0.01 0.0 0.98 0.035], 'String', ...
    sprintf('* landing speed limit = %.1f m/s (%.2fx baseline stall speed) is an ASSUMPTION -- no real requirement exists yet.', V_S_LIMIT, LANDING_SPEED_RATIO), ...
    'EdgeColor', 'none', 'FontSize', 8, 'Color', [0.4 0.4 0.4], 'VerticalAlignment', 'bottom');

%% ---- Plot: MTOM vs payload (weight growth and the MTOW cap) ----
figure('Name','Maximum Payload Weight Growth','Color','w','WindowStyle','docked');
hold on; box on; grid on;
plot(W_pay_grid, MTOM_fixed + W_pay_grid, '-', 'Color', blue, 'LineWidth', 1.8, 'DisplayName', 'MTOM');
yline(MTOW_CAP_KG, '--', 'Color', red, 'LineWidth', 1.2, 'DisplayName', 'MTOW limit, R22 (6 kg)');
xline(W_pay_at_MTOWcap, '-.', 'Color', purple, 'LineWidth', 1.2, 'DisplayName', 'Payload at MTOW cap');
xline(W_pay_max, ':', 'Color', gray, 'LineWidth', 1.3, 'DisplayName', 'Maximum payload (reported)');
plot(W_pay_max, MTOM_fixed + W_pay_max, 'o', 'MarkerFaceColor', orange, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
text(W_pay_max, MTOM_fixed + W_pay_max, sprintf('  %.3f kg payload\n  %.3f kg MTOM', W_pay_max, MTOM_fixed + W_pay_max), ...
    'FontSize', 9, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');
xlabel('Payload mass (kg)'); ylabel('MTOM (kg)');
title('Maximum payload: weight growth against the RFP MTOW cap');
legend('Location', 'southeast');
xlim(xlim_pad);

%% ---- Outputs ----
outputs.MaxPayloadSweep = struct('W_pay_grid', W_pay_grid, 'G', G, 'checks', checks, ...
    'W_pay_cross', W_pay_cross, 'W_pay_max', W_pay_max, 'binding_constraint', binding_name, ...
    'next_constraint', next_name, 'W_pay_next', W_pay_next, 'n_cubes', n_cubes, ...
    'W_pay_at_MTOWcap', W_pay_at_MTOWcap, 'MTOM_fixed', MTOM_fixed);

params.performance.W_pay_max = W_pay_max;

end

function local_panel(idx, x, y, lim, ylab, ttl, W_pay_max, W_pay_MTOWcap, blue, red, gray, purple, limArray, xlim_pad, dataName, limName)
    subplot(2,4,idx); hold on; box on; grid on;
    plot(x, y, '-o', 'Color', blue, 'LineWidth', 1.4, 'MarkerSize', 4, 'DisplayName', dataName);
    if ~isempty(limArray)
        plot(x, limArray, '--', 'Color', red, 'LineWidth', 1.2, 'DisplayName', limName);
    elseif ~isnan(lim)
        yline(lim, '--', 'Color', red, 'LineWidth', 1.2, 'DisplayName', limName);
    end
    xline(W_pay_MTOWcap, '-.', 'Color', purple, 'DisplayName', 'MTOW cap, R22');
    xline(W_pay_max, ':', 'Color', gray, 'LineWidth', 1.3, 'DisplayName', 'Max payload');
    xlabel('Payload mass (kg)'); ylabel(ylab); title(ttl, 'FontSize', 9);
    legend('Location', 'best', 'FontSize', 6);
    if ~isempty(xlim_pad)
        xlim(xlim_pad);
    end
end

function r = local_runCandidate(params, MTOM_fixed, W_pay, figs_keep, V_S_LIMIT)
    % Runs one payload candidate through ONLY the weight-dependent
    % performance scripts (TakeoffPerformance, ClimbPerformance, the turn
    % calculation, MissionSimulation), reusing the FIXED baseline
    % aircraft design (wing area, drag polar, CG, propulsion model)
    % unchanged -- per A9's explicit "keep the airframe, battery, motor
    % and propeller as designed" instruction. Console output is captured
    % via evalc and discarded. Returns NaN/false for anything not reached
    % if an early stage fails or errors, rather than erroring the whole sweep.
    r = struct('MTOM', NaN, 'S_TO', NaN, 'V_S', NaN, 'ROC_max', NaN, 't_climb', NaN, ...
        'CL_turn', NaN, 'CL_limit', NaN, 'P_req_turn', NaN, 'P_avail_turn', NaN, ...
        'laps_possible', NaN, 'P_peak', NaN, 'I_peak', NaN, 'ok', false);

    p = params; % the FIXED baseline design, untouched except for the overrides below
    p.performance.W_pay = W_pay;
    p.performance.MTOM = MTOM_fixed + W_pay;
    p.performance.MTOW = p.performance.MTOM * p.env.g;
    r.MTOM = p.performance.MTOM;

    rho = p.env.rho; S = p.geometry.S_wing; MTOW_i = p.performance.MTOW; g = p.env.g;

    % Stall speed and liftoff speed both derive from MTOW, so both must
    % be refreshed here rather than left at the baseline's now-stale
    % values -- everything else (S, CL_max, CD_0, x_cg_design, the
    % propulsion model) stays exactly as designed.
    r.V_S = sqrt(2*MTOW_i/(rho*S*p.aero.CL_max));
    p.performance.V_S = r.V_S;
    p.performance.V_TO = p.performance.mult_V_TO * r.V_S;
    r.CL_limit = p.aero.CL_max / p.performance.mult_V_TO^2; % same stall-margin convention as CL_R at rotation

    model = p.prop.model; % fixed, already solved once in the baseline params -- not re-solved per candidate

    try
        evalc('[~, p] = TakeoffPerformance(p);');
        evalc('[~, p] = ClimbPerformance(p);');
        local_closeSweepFigs(figs_keep);
        r.S_TO = p.performance.groundRoll_x;
        r.ROC_max = p.performance.ROC_max;
        r.t_climb = p.performance.t_climb;

        % groundRoll_ok=false just means x_LO exceeds the 25 m limit --
        % still a valid, finite distance, not a computation breakdown, so
        % it does NOT stop the rest of this candidate's quantities from
        % being computed (otherwise every panel past "Ground roll" would
        % go blank the moment it fails, even though the team specifically
        % wants each quantity's own trend visible across the whole swept
        % range). ROC_max <= 0 is different: the aircraft genuinely
        % cannot climb at all, so t_climb would be negative/infinite and
        % everything downstream (MissionSimulation's energy integration)
        % becomes meaningless -- that one still stops here.
        if ~(p.performance.ROC_max > 0)
            local_closeSweepFigs(figs_keep);
            return
        end

        V_M = p.performance.V_M; R_req = p.performance.R_req;
        n_turn_i = sqrt(1 + (V_M^2/(g*R_req))^2);
        q_M = 0.5*rho*V_M^2;
        r.CL_turn = n_turn_i*MTOW_i/(q_M*S);
        CD_turn = model.CD_trim_fn(r.CL_turn);
        T_req_turn = q_M*S*CD_turn;
        T_avail_M = model.T_avail_fn(V_M);
        r.P_avail_turn = model.Pelec_avail_fn(V_M);
        if ~isnan(T_avail_M) && T_req_turn <= T_avail_M
            tau_M = model.find_throttle_for_thrust(V_M, T_req_turn);
            if ~isnan(tau_M)
                r.P_req_turn = model.Pelec_grid_fn(tau_M, V_M);
            end
        end

        evalc('[~, p] = MissionSimulation(p);');
        local_closeSweepFigs(figs_keep);
        r.laps_possible = p.performance.mission_laps_possible;

        V_LO = p.performance.groundRoll_V_LO;
        V_ROC = p.performance.V_ROC_max;
        I_TO = model.I_avail_fn(V_LO);
        I_climb = model.I_avail_fn(V_ROC);
        I_straight = NaN; I_turn = NaN;
        V_C = p.performance.V_C;
        q_C = 0.5*rho*V_C^2;
        CL_C = min(MTOW_i/(q_C*S), p.aero.CL_max);
        CD_C = model.CD_trim_fn(CL_C);
        T_req_straight = q_C*S*CD_C;
        T_avail_C = model.T_avail_fn(V_C);
        if ~isnan(T_avail_C) && T_req_straight <= T_avail_C
            tau_C = model.find_throttle_for_thrust(V_C, T_req_straight);
            if ~isnan(tau_C), I_straight = model.I_grid_fn(tau_C, V_C); end
        end
        if ~isnan(r.P_req_turn)
            tau_M2 = model.find_throttle_for_thrust(V_M, T_req_turn);
            if ~isnan(tau_M2), I_turn = model.I_grid_fn(tau_M2, V_M); end
        end
        r.P_peak = max([model.P_cap, r.P_req_turn], [], 'omitnan');
        r.I_peak = max([I_TO, I_climb, I_straight, I_turn], [], 'omitnan');

        r.ok = p.performance.groundRoll_ok && (r.ROC_max > 0) && (r.CL_turn <= r.CL_limit) ...
            && ~isnan(r.P_req_turn) && (r.P_req_turn <= r.P_avail_turn) ...
            && (r.laps_possible >= 3) && (r.P_peak <= 1000) && (r.I_peak <= p.prop.I_motor_limit) ...
            && (r.V_S <= V_S_LIMIT) && (r.S_TO <= p.performance.S_TO);
    catch
        % leave partial results as already filled in; ok stays false
    end
    local_closeSweepFigs(figs_keep);
end

function row = local_logRow(runLabel, iterLabel, W_pay, r)
    row = {runLabel, iterLabel, W_pay, r.MTOM, r.S_TO, r.V_S, r.ROC_max, r.t_climb, ...
        r.CL_turn, r.CL_limit, r.P_req_turn, r.P_avail_turn, r.laps_possible, ...
        r.P_peak, r.I_peak, local_passfail(r.ok)};
end

function s = local_passfail(ok)
    if ok, s = 'PASS'; else, s = 'FAIL'; end
end

function local_writeLog(newRows, runLabel)
    % Overwrites MaxPayloadSweepLog.xlsx (same folder as this script)
    % fresh every run with just this run's candidates -- not accumulated
    % across runs of Main.m.
    colNames = {'Run', 'Iteration', 'W_pay_kg', 'MTOM_kg', 'S_TO_m', 'V_S_mps', 'ROC_max_mps', 't_climb_s', ...
        'CL_turn', 'CL_limit', 'P_req_turn_W', 'P_avail_turn_W', 'LapsPossible', ...
        'PeakPower_W', 'PeakCurrent_A', 'OverallResult'};

    thisFile = mfilename('fullpath');
    xlsxPath = fullfile(fileparts(thisFile), 'MaxPayloadSweepLog.xlsx');

    titleRow = [{sprintf('Deliverable 8 -- Maximum Payload Sweep (last run: %s)', runLabel)}, ...
        repmat({''}, 1, numel(colNames)-1)];
    outTable = [titleRow; colNames; newRows];

    if isfile(xlsxPath)
        % writecell only replaces the named sheet -- any other sheet
        % already in the file (e.g. a leftover default "Sheet1") would
        % otherwise sit there untouched forever. Delete first so the file
        % always ends up with exactly one sheet.
        delete(xlsxPath);
    end
    writecell(outTable, xlsxPath, 'Sheet', 'Deliverable 8');
end

function out = local_ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

function local_closeSweepFigs(figs_keep)
    % Close only figures created since this sweep started -- figs_keep is
    % the snapshot taken at entry, so anything already open (every real
    % deliverable plot from earlier in Main.m) is left untouched.
    figs_now = findall(0, 'Type', 'figure');
    figs_close = setdiff(figs_now, figs_keep);
    for k = 1:numel(figs_close)
        if isvalid(figs_close(k))
            close(figs_close(k));
        end
    end
end
