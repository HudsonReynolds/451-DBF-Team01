function [outputs, params] = CruisePerformance(params)

%% A9 Deliverable 4 -- Cruise
%
% Thrust/power available vs. required, maximum level speed, best-endurance
% and best-range speeds found twice (from the airframe's aerodynamic power
% and from the battery's electrical power) by numerically searching the
% curves -- Deliverable 1 already found the trimmed polar is not a plain
% CD_0+K*CL^2 parabola centred at CL=0, so the textbook closed-form speeds
% would be wrong here; this searches the actual arrays instead, per A9's
% instruction.

if ~isfield(params.prop, 'model')
    error('CruisePerformance:noModel', 'Run PropulsionDragModel(params) first.');
end
model = params.prop.model;

rho = params.env.rho;
S = params.geometry.S_wing;
MTOW = params.performance.MTOW;
V_S = params.performance.V_S;

V_sweep = linspace(V_S, model.V_grid(end), 300);
q = 0.5*rho*V_sweep.^2;
CL = min(MTOW ./ (q*S), params.aero.CL_max);
CD = model.CD_trim_fn(CL);

T_req = q*S.*CD;            % level flight: T = D
T_avail = model.T_avail_fn(V_sweep);

%% ---- Maximum level speed: where available crosses required ----
diff_T = T_avail - T_req;
i_cross = find(diff_T(1:end-1) >= 0 & diff_T(2:end) < 0, 1, 'first');
if isempty(i_cross)
    V_max_level = NaN;
    warning('CruisePerformance:noCrossing', 'Thrust available never falls below thrust required in the swept range -- cannot bound max level speed.');
else
    V_max_level = interp1(diff_T(i_cross:i_cross+1), V_sweep(i_cross:i_cross+1), 0);
end

V_max_level_A8 = params.performance.V_max_level; % the coarser, V-independent-RPM estimate PropulsionSizing.m made (A7/A8)
fprintf('\n--- Cruise Performance (Deliverable 4) ---\n');
fprintf('  Maximum level speed (refined model): %.2f m/s  (A8''s PropulsionSizing.m estimate: %.2f m/s)\n', V_max_level, V_max_level_A8);
if abs(V_max_level - V_max_level_A8) > 0.5
    fprintf('  >>> FINDING: refined V_max_level differs from the one A8''s V_NE check already used by %.2f m/s -- A8 ran earlier in Main.m with the older value.\n', ...
        V_max_level - V_max_level_A8);
end
params.performance.V_max_level = V_max_level; % update for anything that runs after this point

%% ---- Electrical power required: throttle search at each V ----
P_req_aero = T_req .* V_sweep;
P_req_elec = nan(size(V_sweep));
for j = 1:numel(V_sweep)
    if isnan(T_avail(j)) || T_req(j) > T_avail(j)
        continue % beyond max level speed -- no level-flight solution, report no result
    end
    tau_level = model.find_throttle_for_thrust(V_sweep(j), T_req(j));
    if ~isnan(tau_level)
        P_req_elec(j) = model.Pelec_grid_fn(tau_level, V_sweep(j));
    end
end
P_avail_elec = model.Pelec_avail_fn(V_sweep);
eta_combined = P_req_aero ./ P_req_elec;

%% ---- Best endurance / best range, found twice, by search ----
[~, i_end_aero] = min(P_req_aero);
[~, i_end_elec] = min(P_req_elec);
[~, i_rng_aero] = max(V_sweep ./ P_req_aero);
[~, i_rng_elec] = max(V_sweep ./ P_req_elec);

V_endurance_aero = V_sweep(i_end_aero); V_endurance_elec = V_sweep(i_end_elec);
V_range_aero = V_sweep(i_rng_aero);     V_range_elec = V_sweep(i_rng_elec);

fprintf('  Best endurance: %.2f m/s (airframe power), %.2f m/s (electrical power)\n', V_endurance_aero, V_endurance_elec);
fprintf('  Best range:     %.2f m/s (airframe power), %.2f m/s (electrical power)\n', V_range_aero, V_range_elec);
fprintf('  Difference is propulsive/motor/ESC efficiency changing with operating point -- it shifts the battery-draw minimum away from the pure-aerodynamic minimum.\n');

%% ---- Endurance and range, from usable battery energy ----
E_usable = params.performance.usableBatteryEnergy_J;
P_elec_endurance = P_req_elec(i_end_elec);
P_elec_range = P_req_elec(i_rng_elec);

endurance_min = E_usable / P_elec_endurance / 60;
range_km = E_usable / P_elec_range * V_range_elec / 1000;

fprintf('  Usable battery energy: %.0f kJ (%.0f%% of pack)\n', E_usable/1000, 100*params.prop.useableCapacity*params.prop.temp_derate);
fprintf('  >>> Maximum endurance: %.1f min at %.2f m/s.  Maximum range: %.2f km at %.2f m/s.\n', endurance_min, V_endurance_elec, range_km, V_range_elec);

%% ---- Plot 1: thrust available and required, matching the A9 example ----
blue = [0.00 0.45 0.74]; orange = [0.85 0.33 0.10]; red = [0.80 0.00 0.00];

figure('Name','Thrust Available and Required','Color','w','WindowStyle','docked');
hold on; box on; grid on;
plot(V_sweep, T_avail, '-', 'Color', orange, 'LineWidth', 1.8, 'DisplayName', 'Thrust available, full throttle');
plot(V_sweep, T_req, '--', 'Color', blue, 'LineWidth', 1.8, 'DisplayName', 'Thrust required, level flight');
xline(V_S, ':', 'Color', [0.3 0.3 0.3], 'DisplayName', 'Stall speed');
if ~isnan(V_max_level)
    plot(V_max_level, interp1(V_sweep, T_req, V_max_level), 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
    text(V_max_level, interp1(V_sweep, T_req, V_max_level), sprintf('  maximum level speed %.1f m/s', V_max_level), ...
        'FontSize', 9, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
end
xlabel('True airspeed (m/s)'); ylabel('Thrust (N)');
title('Thrust available and thrust required');
legend('Location', 'best');
% The data starts exactly at V_S, so the stall-speed line would otherwise
% sit flush with the left axis and be invisible behind it.
xlim([V_S - 0.03*(V_sweep(end)-V_S), V_sweep(end)]);

%% ---- Plot 2: power required (+ available, + efficiency), matching the A9 example ----
figure('Name','Power Required','Color','w','WindowStyle','docked');
hold on; box on; grid on;
yyaxis left
plot(V_sweep, P_req_aero, '-', 'Color', [0.00 0.60 0.50], 'LineWidth', 1.8, 'DisplayName', 'Power required at the airframe');
plot(V_sweep, P_req_elec, '--', 'Color', blue, 'LineWidth', 1.8, 'DisplayName', 'Electrical power drawn from the battery');
plot(V_sweep, P_avail_elec, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.3, 'DisplayName', 'Electrical power available');
ylabel('Power (W)');
ax = gca; ax.YColor = 'k';
xline(V_S, ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off');
plot(V_endurance_aero, P_req_aero(i_end_aero), 'o', 'MarkerFaceColor', [0.00 0.60 0.50], 'MarkerEdgeColor', 'k', 'MarkerSize', 7, 'HandleVisibility', 'off');
plot(V_endurance_elec, P_req_elec(i_end_elec), 'o', 'MarkerFaceColor', blue, 'MarkerEdgeColor', 'k', 'MarkerSize', 7, 'HandleVisibility', 'off');
if abs(V_endurance_aero - V_endurance_elec) < 0.5
    % Same speed for both (common when it lands right at stall speed,
    % the edge of the sweep) -- one shared label instead of two
    % overlapping ones.
    text(V_endurance_aero, max(P_req_aero(i_end_aero), P_req_elec(i_end_elec)), sprintf('  %.1f m/s (both)', V_endurance_aero), ...
        'FontSize', 8.5, 'VerticalAlignment', 'bottom');
else
    text(V_endurance_aero, P_req_aero(i_end_aero), sprintf('  %.1f m/s', V_endurance_aero), 'FontSize', 8.5, 'VerticalAlignment', 'top');
    text(V_endurance_elec, P_req_elec(i_end_elec), sprintf('  %.1f m/s', V_endurance_elec), 'FontSize', 8.5, 'VerticalAlignment', 'bottom');
end

yyaxis right
plot(V_sweep, eta_combined, '-.', 'Color', orange, 'LineWidth', 1.4, 'DisplayName', 'Combined propulsive efficiency (right axis)');
ylabel('Efficiency (-)');
ax.YColor = 'k';
ylim([0 1]);

xlabel('True airspeed (m/s)');
title('Power required against airspeed');
legend('Location', 'best');
% Same reason as the thrust plot above: give the stall-speed line (and
% the endurance-point labels, which also sit right at V_S) room on the left.
xlim([V_S - 0.03*(V_sweep(end)-V_S), V_sweep(end)]);

%% ---- Outputs ----
outputs.CruisePerformance = struct('V', V_sweep, 'T_avail', T_avail, 'T_req', T_req, 'V_max_level', V_max_level, ...
    'P_req_aero', P_req_aero, 'P_req_elec', P_req_elec, 'P_avail_elec', P_avail_elec, 'eta_combined', eta_combined, ...
    'V_endurance_aero', V_endurance_aero, 'V_endurance_elec', V_endurance_elec, ...
    'V_range_aero', V_range_aero, 'V_range_elec', V_range_elec, ...
    'endurance_min', endurance_min, 'range_km', range_km, 'E_usable_J', E_usable);

params.performance.V_endurance = V_endurance_elec;
params.performance.V_range = V_range_elec;
params.performance.endurance_min = endurance_min;
params.performance.range_km = range_km;

end
