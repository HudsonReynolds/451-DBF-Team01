function [outputs, params] = MaxPayloadSweep(params)

%% A9 Deliverable 8 -- Maximum Payload
%
% Per A9's own instructions: sweep payload mass at 10+ values up to the
% RFP maximum take-off weight (6 kg, R22), re-running every weight-
% dependent analysis at each mass (not scaling the nominal result), and
% track six things against their own limit:
%   1. Ground roll, against the RFP limit (R1, 25 m).
%   2. Stall speed, against the landing requirement (SizingParams.xlsx
%      V_S_landing_limit -- an invented placeholder; see item below).
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
% That's 8 individual pass/fail checks once the turn and the peak-draw
% items are split into their two parts each. The maximum payload is
% whichever one fails at the LOWEST payload; MTOW (6 kg) is only the
% sweep's own upper bound, not itself one of the 6 listed quantities, so
% it is not treated as a constraint here, unlike an earlier version of
% this script.
%
% ASSUMPTION (SizingParams.xlsx, V_S_landing_limit): no numeric landing
% speed requirement exists anywhere in this project's prior assignments
% or the team's requirements document (R5 only says "lands on grass
% without damage," no speed). Per the team's direction, a placeholder of
% 13 m/s (roughly 25% above the baseline self-consistent stall speed) was
% added and clearly flagged for the team to replace with a real number.
%
% Re-runs InitialCalcs/DragBuildUp/calcAero/VehicleWeightEstimation (the
% weight/aero loop), A5B (needed because PropulsionDragModel's trim drag
% uses x_cg_design, which only A5B sets), PropulsionDragModel,
% TakeoffPerformance, ClimbPerformance, and MissionSimulation at every
% candidate payload -- ServoSizing and PropulsionSizing are skipped
% because nothing in this particular chain reads their outputs. Every
% candidate's console output is captured via evalc and discarded
% (warnings silenced for the sweep's duration); the full per-candidate
% numbers are logged to MaxPayloadSweepLog.xlsx instead. Each candidate's
% throwaway figures are closed immediately; only this script's own
% summary figure survives.

if ~isfield(params.performance, 'W_pay')
    error('MaxPayloadSweep:noBaseline', 'params must already contain a baseline W_pay (run the normal Main.m pipeline first).');
end
if ~isfield(params.performance, 'V_S_landing_limit')
    error('MaxPayloadSweep:noLandingLimit', 'SizingParams.xlsx needs a V_S_landing_limit entry (performance) -- see this script''s header.');
end

figs_keep = findall(0, 'Type', 'figure');
warnState = warning('off', 'all');
cleanupWarn = onCleanup(@() warning(warnState));

%% ---- Find the sweep's upper bound: payload at MTOM = 6 kg (R22) ----
MTOW_CAP_KG = 6.0;
W_pay_hi_search = params.performance.W_pay;
MTOM_at_hi = local_weightOnly(params, W_pay_hi_search);
iter_guard = 0;
while MTOM_at_hi < MTOW_CAP_KG && iter_guard < 30
    W_pay_hi_search = W_pay_hi_search * 1.5;
    MTOM_at_hi = local_weightOnly(params, W_pay_hi_search);
    iter_guard = iter_guard + 1;
end
lo_b = 0; hi_b = W_pay_hi_search;
for b = 1:40
    mid_b = 0.5*(lo_b+hi_b);
    if local_weightOnly(params, mid_b) < MTOW_CAP_KG, lo_b = mid_b; else, hi_b = mid_b; end
end
W_pay_at_MTOWcap = lo_b;

N_PTS = 12;
W_pay_grid = linspace(0.05, W_pay_at_MTOWcap, N_PTS);

fprintf('\n--- Maximum Payload Sweep (Deliverable 8) ---\n');
fprintf('  Sweeping W_pay from %.3f to %.3f kg over %d points (payload at MTOM = %.1f kg, R22).\n', ...
    W_pay_grid(1), W_pay_grid(end), N_PTS, MTOW_CAP_KG);

runLabel = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm'));
logRows = cell(0, 16);

G = struct('MTOM', nan(1,N_PTS), 'S_TO', nan(1,N_PTS), 'V_S', nan(1,N_PTS), ...
    'ROC_max', nan(1,N_PTS), 't_climb', nan(1,N_PTS), ...
    'CL_turn', nan(1,N_PTS), 'CL_limit', nan(1,N_PTS), ...
    'P_req_turn', nan(1,N_PTS), 'P_avail_turn', nan(1,N_PTS), ...
    'laps_possible', nan(1,N_PTS), 'P_peak', nan(1,N_PTS), 'I_peak', nan(1,N_PTS), ...
    'ok', false(1,N_PTS));

for i = 1:N_PTS
    r = local_runCandidate(params, W_pay_grid(i), figs_keep);
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
V_S_LIMIT = params.performance.V_S_landing_limit;

checks = struct( ...
    'name', {'Ground roll', 'Stall speed', 'Max rate of climb', 'Turn CL (stall margin)', ...
              'Turn power required', 'Laps possible', 'Peak electrical power', 'Peak current'}, ...
    'value', {G.S_TO, G.V_S, G.ROC_max, G.CL_turn, G.P_req_turn - G.P_avail_turn, ...
              G.laps_possible, G.P_peak, G.I_peak}, ...
    'limit', {S_TO_LIMIT, V_S_LIMIT, 0, NaN, 0, 3, P_CAP_RFP, I_CAP}, ...
    'sense', {'max', 'max', 'min', 'max', 'max', 'min', 'max', 'max'}); % 'max': fails when value > limit; 'min': fails when value < limit
checks(4).limit = NaN; % filled per-point below (CL_R varies negligibly but compute properly)
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

[W_pay_max, i_binding] = min(W_pay_cross);
binding_name = checks(i_binding).name;
hitMTOWfallback = isinf(W_pay_max);

% Next-binding constraint: the smallest crossing strictly greater than
% the binding one (not just the second-smallest overall, which could
% equal the binding value itself under ties, and not just "first finite,"
% which could re-select the binding one when it is itself finite).
higherCrossings = W_pay_cross(W_pay_cross > W_pay_max);
if isempty(higherCrossings)
    W_pay_next = NaN; i_next = NaN;
else
    W_pay_next = min(higherCrossings);
    i_next = find(W_pay_cross == W_pay_next, 1);
end

if hitMTOWfallback
    % None of the 6 analyzed quantities ever fails before the sweep's own
    % upper bound (payload at MTOM = 6 kg, R22) -- MTOW itself is what
    % actually stops payload growth here, not any of the 6 listed
    % analyses, which all still have margin right up to that cap.
    W_pay_max = W_pay_at_MTOWcap;
    binding_name = 'MTOW cap (R22, 6 kg) -- none of the 6 analyzed quantities bind first';
    fprintf('  >>> None of the 6 analyzed quantities (ground roll, stall speed, climb, turn, laps, peak power/current)\n');
    fprintf('      fails before MTOM reaches 6 kg -- the RFP weight cap itself is the limit here, not any of them.\n');
end

n_cubes = floor(W_pay_max / 0.52); % R14: 520 g per payload cube

fprintf('  >>> Maximum payload: %.3f kg, set by: %s (%d whole payload cubes, R14 @ 0.52 kg each)\n', ...
    W_pay_max, binding_name, n_cubes);
fprintf('  Margins on the other constraints at W_pay = %.3f kg:\n', W_pay_max);
for c = 1:numel(checks)
    if ~hitMTOWfallback && c == i_binding, continue, end
    fprintf('    %-24s binds at %.3f kg%s\n', checks(c).name, W_pay_cross(c), ...
        local_ternary(isinf(W_pay_cross(c)), ' (never, within the swept range)', ''));
end
if ~isnan(i_next)
    fprintf('  Next-binding constraint if %s is relieved: %s, at %.3f kg\n', binding_name, checks(i_next).name, W_pay_next);
else
    fprintf('  No other constraint binds within the swept range -- raising MTOW itself is the only way to carry more payload.\n');
end

%% ---- Plot: all 8 checks against payload, 2x4 panels ----
blue = [0.00 0.45 0.74]; orange = [0.85 0.33 0.10]; red = [0.80 0.00 0.00]; gray = [0.3 0.3 0.3];

% The swept range always ends exactly at W_pay_max when nothing else
% binds first (and often ends very close to it otherwise), so with no
% margin the vertical "max payload" line sits flush on each panel's right
% edge, hidden behind the axis spine -- same fix as the earlier takeoff/
% climb plots this session. Give every panel's x-axis a little headroom.
xlim_pad = [0, max(W_pay_grid(end), W_pay_max)*1.08];

% No explicit 'Position' here -- Setup.m already sets every figure to
% dock by default, and combining 'WindowStyle','docked' with an explicit
% Position pops the figure out as its own separate window instead of
% joining the rest in the same docked tab group (same conflict fixed
% elsewhere earlier this session). The 2x4 panel layout below still
% renders fine at whatever size the docked group happens to be; the user
% can resize that group interactively like any other docked figure.
figure('Name','Maximum Payload Sweep','Color','w','WindowStyle','docked');

local_panel(1, W_pay_grid, G.S_TO, S_TO_LIMIT, 'max', 'Ground roll (m)', 'Ground roll vs RFP limit', W_pay_max, blue, red, gray, [], xlim_pad);
local_panel(2, W_pay_grid, G.V_S, V_S_LIMIT, 'max', 'Stall speed (m/s)', 'Stall speed vs landing limit*', W_pay_max, blue, red, gray, [], xlim_pad);
local_panel(3, W_pay_grid, G.ROC_max, 0, 'min', 'Max rate of climb (m/s)', 'Max ROC vs 0 (climb feasibility)', W_pay_max, blue, red, gray, [], xlim_pad);
local_panel(4, W_pay_grid, G.CL_turn, NaN, 'max', 'Turn C_L (-)', 'Turn CL vs stall-margined limit', W_pay_max, blue, red, gray, G.CL_limit, xlim_pad);

subplot(2,4,5); hold on; box on; grid on;
plot(W_pay_grid, G.P_avail_turn, '-', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', 'Available');
plot(W_pay_grid, G.P_req_turn, '--', 'Color', blue, 'LineWidth', 1.6, 'DisplayName', 'Required');
xline(W_pay_max, ':', 'Color', gray, 'HandleVisibility', 'off');
xlabel('Payload mass (kg)'); ylabel('Turn power (W)'); title('Turn power required vs available'); legend('Location','best','FontSize',7);
xlim(xlim_pad);

local_panel(6, W_pay_grid, G.laps_possible, 3, 'min', 'Laps possible (-)', 'Laps possible vs 3 required', W_pay_max, blue, red, gray, [], xlim_pad);
local_panel(7, W_pay_grid, G.P_peak, P_CAP_RFP, 'max', 'Peak electrical power (W)', 'Peak power vs RFP limit (R21)', W_pay_max, blue, red, gray, [], xlim_pad);
local_panel(8, W_pay_grid, G.I_peak, I_CAP, 'max', 'Peak current (A)', 'Peak current vs motor/controller limit', W_pay_max, blue, red, gray, [], xlim_pad);

if hitMTOWfallback
    title_binding = 'MTOW cap (R22) -- nothing else binds first';
else
    title_binding = binding_name;
end
sgtitle(sprintf('Maximum payload %.3f kg (%d cubes), set by: %s', W_pay_max, n_cubes, title_binding), ...
    'FontWeight', 'bold', 'FontSize', 13);
annotation('textbox', [0.01 0.0 0.98 0.035], 'String', ...
    sprintf('* landing speed limit = %.0f m/s is an ASSUMPTION (SizingParams.xlsx V_S_landing_limit) -- no real requirement exists yet', V_S_LIMIT), ...
    'EdgeColor', 'none', 'FontSize', 8, 'Color', [0.4 0.4 0.4], 'VerticalAlignment', 'bottom');

%% ---- Outputs ----
if isnan(i_next)
    next_constraint_name = 'none within the swept range';
else
    next_constraint_name = checks(i_next).name;
end
outputs.MaxPayloadSweep = struct('W_pay_grid', W_pay_grid, 'G', G, 'checks', checks, ...
    'W_pay_cross', W_pay_cross, 'W_pay_max', W_pay_max, 'binding_constraint', binding_name, ...
    'next_constraint', next_constraint_name, 'W_pay_next', W_pay_next, 'n_cubes', n_cubes, ...
    'W_pay_at_MTOWcap', W_pay_at_MTOWcap);

params.performance.W_pay_max = W_pay_max;

end

function panelOut = local_panel(idx, x, y, lim, sense, ylab, ttl, W_pay_max, blue, red, gray, limArray, xlim_pad)
    subplot(2,4,idx); hold on; box on; grid on;
    plot(x, y, '-o', 'Color', blue, 'LineWidth', 1.4, 'MarkerSize', 4, 'HandleVisibility', 'off');
    if nargin >= 12 && ~isempty(limArray)
        plot(x, limArray, '--', 'Color', red, 'LineWidth', 1.2, 'HandleVisibility', 'off');
    elseif ~isnan(lim)
        yline(lim, '--', 'Color', red, 'LineWidth', 1.2, 'HandleVisibility', 'off');
    end
    xline(W_pay_max, ':', 'Color', gray, 'HandleVisibility', 'off');
    xlabel('Payload mass (kg)'); ylabel(ylab); title(ttl, 'FontSize', 9);
    if nargin >= 13 && ~isempty(xlim_pad)
        xlim(xlim_pad);
    end
    panelOut = [];
end

function MTOM = local_weightOnly(params, W_pay)
    % Fast path for finding the MTOW=6kg sweep bound: only the
    % lightweight weight/aero convergence loop, none of the heavy
    % propulsion/performance chain.
    p = params;
    p.performance.W_pay = W_pay;
    MTOM_guess = p.performance.MTOM;
    err = 1; iter = 0;
    while err > 0.001 && iter < 50
        p = InitialCalcs(p);
        p = DragBuildUp(p);
        p = calcAero(p);
        p = VehicleWeightEstimation(p);
        MTOM_new = p.performance.MTOM;
        err = abs((MTOM_guess - MTOM_new) / MTOM_guess);
        MTOM_guess = MTOM_new;
        iter = iter + 1;
    end
    MTOM = p.performance.MTOM;
end

function r = local_runCandidate(params, W_pay, figs_keep)
    % Runs one payload candidate through the full weight/stability/
    % propulsion/performance chain, with every called script's console
    % output captured via evalc and discarded. Returns NaN/false for
    % anything not reached if an early stage fails or errors, rather than
    % erroring the whole sweep.
    r = struct('MTOM', NaN, 'S_TO', NaN, 'V_S', NaN, 'ROC_max', NaN, 't_climb', NaN, ...
        'CL_turn', NaN, 'CL_limit', NaN, 'P_req_turn', NaN, 'P_avail_turn', NaN, ...
        'laps_possible', NaN, 'P_peak', NaN, 'I_peak', NaN, 'ok', false);

    p = params;
    p.performance.W_pay = W_pay;

    MTOM_guess = p.performance.MTOM;
    err = 1; iter = 0;
    while err > 0.001 && iter < 50
        p = InitialCalcs(p);
        p = DragBuildUp(p);
        p = calcAero(p);
        p = VehicleWeightEstimation(p);
        MTOM_new = p.performance.MTOM;
        err = abs((MTOM_guess - MTOM_new) / MTOM_guess);
        MTOM_guess = MTOM_new;
        iter = iter + 1;
    end
    r.MTOM = p.performance.MTOM;
    if err > 0.001
        return
    end

    try
        evalc('p = A5B(p);');
        evalc('[~, p] = PropulsionDragModel(p);');
        local_closeSweepFigs(figs_keep);
        model = p.prop.model;

        rho = p.env.rho; S = p.geometry.S_wing; MTOW_i = p.performance.MTOW; g = p.env.g;
        r.V_S = sqrt(2*MTOW_i/(rho*S*p.aero.CL_max));
        r.CL_limit = p.aero.CL_max / p.performance.mult_V_TO^2; % same stall-margin convention as CL_R at rotation

        evalc('[~, p] = TakeoffPerformance(p);');
        evalc('[~, p] = ClimbPerformance(p);');
        local_closeSweepFigs(figs_keep);
        r.S_TO = p.performance.groundRoll_x;
        r.ROC_max = p.performance.ROC_max;
        r.t_climb = p.performance.t_climb;

        if ~p.performance.groundRoll_ok || ~(p.performance.ROC_max > 0)
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
            && (r.V_S <= p.performance.V_S_landing_limit) && (r.S_TO <= p.performance.S_TO);
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
