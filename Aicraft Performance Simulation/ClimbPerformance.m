function [outputs, params] = ClimbPerformance(params)

%% A9 Deliverable 3 -- Climb
%
% Rate of climb from excess power, ROC(V) = (T_avail(V) - D(V))*V / MTOW,
% using PropulsionDragModel.m's capped thrust-available model and the
% trimmed drag polar at the level-flight-equivalent CL (standard small-
% climb-angle approximation: CL = MTOW/(q*S), i.e. climb angle's cosine
% correction to lift is neglected -- ASSUMPTION, reasonable for the
% shallow climb angles this class of aircraft flies).

if ~isfield(params.prop, 'model')
    error('ClimbPerformance:noModel', 'Run PropulsionDragModel(params) first.');
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
D = q*S.*CD;
T = model.T_avail_fn(V_sweep);

ROC = (T - D) .* V_sweep / MTOW;

% Truncate where thrust data runs out (NaN) or where ROC first crosses
% zero -- "from the stall speed to the speed where the excess power runs
% out," matching the A9 example figure exactly.
valid = ~isnan(ROC);
i_last_valid = find(valid, 1, 'last');
V_sweep = V_sweep(1:i_last_valid);
ROC = ROC(1:i_last_valid);

i_zero = find(ROC <= 0, 1, 'first');
if ~isempty(i_zero) && i_zero > 1
    % interpolate the true zero-crossing rather than stopping at the grid point
    V_zero = interp1(ROC(i_zero-1:i_zero), V_sweep(i_zero-1:i_zero), 0);
    V_sweep = [V_sweep(1:i_zero-1), V_zero];
    ROC = [ROC(1:i_zero-1), 0];
end

[ROC_max, i_max] = max(ROC);
V_ROC_max = V_sweep(i_max);

fprintf('\n--- Climb Performance (Deliverable 3) ---\n');
fprintf('  Maximum rate of climb: %.2f m/s at %.1f m/s true airspeed\n', ROC_max, V_ROC_max);

%% ---- Time and horizontal distance to the RFP altitude ----
% ASSUMPTION: climbs at the constant best-ROC speed/angle the whole way
% (same simplified constant-speed-segment convention InitialCalcs.m/
% VehicleWeightEstimation.m already use for every other flight phase --
% T_CL there is built the same way), rather than an optimal climb schedule.
climb_alt = params.performance.climb_alt;
t_climb = climb_alt / ROC_max;
V_horizontal = sqrt(max(V_ROC_max^2 - ROC_max^2, 0));
x_climb = V_horizontal * t_climb;

fprintf('  Time to RFP altitude (%.0f m): %.2f s, horizontal distance %.1f m (constant best-ROC-speed climb, ASSUMPTION)\n', ...
    climb_alt, t_climb, x_climb);

%% ---- Plot: rate of climb vs airspeed, matching the A9 example ----
blue = [0.00 0.45 0.74]; red = [0.80 0.00 0.00];

figure('Name','Climb Performance','Color','w','WindowStyle','docked');
hold on; box on; grid on;

plot(V_sweep, ROC, '-', 'Color', blue, 'LineWidth', 1.8, 'DisplayName', 'Rate of climb at full throttle');
xline(V_S, ':', 'Color', [0.3 0.3 0.3], 'DisplayName', 'Stall speed');
plot(V_ROC_max, ROC_max, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
text(V_ROC_max, ROC_max, sprintf('  maximum %.1f m/s at %.1f m/s', ROC_max, V_ROC_max), ...
    'FontSize', 9, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');

xlabel('True airspeed (m/s)');
ylabel('Rate of climb (m/s)');
title('Rate of climb against airspeed');
legend('Location', 'best');
ylim([0, ROC_max*1.25]);
% The data starts exactly at V_S, so the stall-speed line would otherwise
% sit flush with the left axis and be invisible behind it -- give it a
% sliver of margin to its left so it actually shows as a line.
xlim([V_S - 0.03*(V_sweep(end)-V_S), V_sweep(end)]);

%% ---- Outputs ----
outputs.ClimbPerformance = struct('V', V_sweep, 'ROC', ROC, 'ROC_max', ROC_max, 'V_ROC_max', V_ROC_max, ...
    't_climb', t_climb, 'x_climb', x_climb, 'climb_alt', climb_alt);

params.performance.ROC_max = ROC_max;
params.performance.V_ROC_max = V_ROC_max;
params.performance.t_climb = t_climb;

end
