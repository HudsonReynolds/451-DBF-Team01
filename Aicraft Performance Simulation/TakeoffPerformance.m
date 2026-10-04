function [outputs, params] = TakeoffPerformance(params)

%% A9 Deliverable 2 -- Take-off
%
% Integrates the ground roll in time (forward Euler), using thrust from
% PropulsionDragModel.m's capped model (Deliverable 1) at full throttle
% and the trimmed drag polar (also Deliverable 1) at the rotation CL
% already established in InitialCalcs.m (CL_TO = CL_C, "no flaps, same as
% cruise"). Reports ground roll against the RFP limit (S_TO) and plots
% velocity/distance vs time with lift-off marked, matching the A9 example
% figure's style.

if ~isfield(params.prop, 'model')
    error('TakeoffPerformance:noModel', 'Run PropulsionDragModel(params) first -- this script needs its capped thrust-available model.');
end
model = params.prop.model;

g = params.env.g;
rho = params.env.rho;
S = params.geometry.S_wing;
MTOM = params.performance.MTOM;
MTOW = params.performance.MTOW;

% A9 Deliverable 2: "Integrate the ground roll with a rolling-resistance
% coefficient of 0.04." Stated explicitly by the assignment, not a team
% assumption -- NOT the same as the existing env.mu_TO=0.05 used
% elsewhere in the codebase (A7 static-thrust sizing, A8 tail-dragger
% ground-roll torque), so it is intentionally not read from Sheet1.
mu_roll = 0.04;

dt = 0.01; % s -- time step, stated per A9's instruction

CL_TO = params.aero.CL_TO; % "no flaps, same as cruise" (InitialCalcs.m)
V_TO  = params.performance.V_TO; % rotation/lift-off speed, mult_V_TO * V_S

%% ---- Forward-Euler integration ----
t = 0; V = 0; x = 0;
t_hist = t; V_hist = V; x_hist = x;

% The ground roll itself ends at lift-off (V >= V_TO) -- that's still what
% gets reported and checked against the RFP limit below. But the plot
% keeps integrating past that point, up to a fixed 12 s window, purely so
% the curves have room to visually separate from the lift-off marker
% instead of both ending flush at it. Once airborne the wheels are no
% longer on the runway, so the rolling-resistance term is dropped from
% that point on -- this post-lift-off segment is a simplified constant-CL
% extrapolation of the same thrust/drag model, NOT a real climb-out
% trajectory (no climb angle, no ClimbPerformance.m physics); it exists
% for plot continuity only.
T_PLOT_END = 12.0; % s
liftoff_found = false;
t_LO = NaN; V_LO = NaN; x_LO = NaN;

max_steps = round(T_PLOT_END/dt) + 1;
for k = 1:max_steps
    q = 0.5*rho*V^2;
    D = q*S*model.CD_trim_fn(CL_TO);
    L = q*S*CL_TO;
    T = model.T_avail_fn(V);
    if isnan(T)
        error('TakeoffPerformance:noThrust', 'No valid thrust-available point at V=%.2f m/s -- outside the propulsion model''s range.', V);
    end

    if liftoff_found
        friction = 0; % wheels off the ground -- no more rolling resistance
    else
        friction = mu_roll*(MTOW - L);
    end
    a = (T - D - friction) / MTOM;
    if a <= 0
        warning('TakeoffPerformance:noAccel', 'Net acceleration reached zero at V=%.2f m/s, before V_TO=%.2f m/s -- aircraft cannot complete the roll as modelled.', V, V_TO);
        break
    end

    V = V + a*dt;
    x = x + V*dt;
    t = t + dt;

    t_hist(end+1) = t; %#ok<AGROW>
    V_hist(end+1) = V; %#ok<AGROW>
    x_hist(end+1) = x; %#ok<AGROW>

    if ~liftoff_found && V >= V_TO
        liftoff_found = true;
        t_LO = t; V_LO = V; x_LO = x; % record the real lift-off instant, then keep going for the plot
    end

    if t >= T_PLOT_END
        break
    end
end

S_TO_limit = params.performance.S_TO;
ok_S_TO = x_LO <= S_TO_limit;

fprintf('\n--- Take-off Performance (Deliverable 2) ---\n');
fprintf('  Time step:          dt = %.3f s\n', dt);
fprintf('  Rolling friction:   mu = %.2f  (A9-stated, not env.mu_TO = %.2f)\n', mu_roll, params.env.mu_TO);
fprintf('  Lift-off CL:        CL_TO = %.3f  (no flaps)\n', CL_TO);
fprintf('  Lift-off time:      t_LO = %.2f s\n', t_LO);
fprintf('  Lift-off distance:  x_LO = %.2f m\n', x_LO);
fprintf('  Lift-off speed:     V_LO = %.2f m/s\n', V_LO);
fprintf('  Ground roll vs RFP limit (%.1f m): %s\n', S_TO_limit, local_ternary(ok_S_TO, 'PASS', 'FAIL'));

%% ---- Plot: velocity and distance vs time, matching the A9 example ----
blue = [0.00 0.45 0.74]; orange = [0.85 0.33 0.10];

figure('Name','Take-off Ground Roll','Color','w','WindowStyle','docked');
hold on; box on; grid on;

yyaxis left
plot(t_hist, V_hist, '-', 'Color', blue, 'LineWidth', 1.8, 'DisplayName', 'Velocity (left axis, solid)');
ylabel('Velocity (m/s)');
ax = gca; ax.YColor = blue;

yyaxis right
plot(t_hist, x_hist, '--', 'Color', orange, 'LineWidth', 1.8, 'DisplayName', 'Distance (right axis, dashed)');
ylabel('Ground distance (m)');
ax.YColor = orange;

yyaxis left
plot([t_LO t_LO], [0 V_LO], ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off');
plot(t_LO, V_LO, 'o', 'MarkerFaceColor', [0.80 0.00 0.00], 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
% Multi-line label as a cell array (one string per line), not an
% embedded "\n" -- the default LaTeX interpreter doesn't treat a literal
% newline inside one string as a line break, and warns (same issue fixed
% earlier in ControlSurfaceSizing.m).
text(t_LO, V_LO*0.55, {'  lift-off', sprintf('  %.1f s, %.1f m', t_LO, x_LO), sprintf('  $V_{LO}$ = %.1f m/s', V_LO)}, ...
    'FontSize', 9, 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');

xlabel('Time (s)');
title('Take-off ground roll');
legend('Location', 'northwest');
% Plotting out to T_PLOT_END puts lift-off at roughly t_LO/12 of the way
% across the window instead of right at the data's edge, so the marker
% naturally lands away from the corner -- no y-axis override needed to
% separate it from the curves' endpoints. The x-axis DOES need pinning,
% though: MATLAB's default autoscaling rounds the 12.0 s data extent up
% to a "nice" 14 s, leaving 2 s of dead space after the last sample --
% the curves then look like they're abruptly chopped off mid-chart
% instead of simply running to the edge of the plotted window.
xlim([0, t_hist(end)]);

%% ---- Outputs ----
outputs.TakeoffPerformance = struct('t', t_hist, 'V', V_hist, 'x', x_hist, ...
    't_LO', t_LO, 'x_LO', x_LO, 'V_LO', V_LO, 'CL_TO', CL_TO, 'mu_roll', mu_roll, 'dt', dt, ...
    'S_TO_limit', S_TO_limit, 'ok_S_TO', ok_S_TO);

params.performance.groundRoll_x = x_LO;
params.performance.groundRoll_t = t_LO;
params.performance.groundRoll_ok = ok_S_TO;

end

function out = local_ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
