function [outputs, params] = TakeOffRotation(params)

%% A8 Deliverable 4 -- Take-off rotation

%% Initializations

rho = params.env.rho;
g   = params.env.g;
S   = params.geometry.S_wing;
c   = params.geometry.c_wing;

% Mass table is the source of truth for structural analysis (team
% decision): use its own sum as the aircraft weight here, same as
% WingLoads.m, TailLoads.m and FuselageLoads.m.
massTable = readtable('SizingParams.xlsx', 'Sheet', 'MassBudget');
MTOM = sum(massTable.Mass_g)/1000; % kg
MTOW = MTOM * g; % N
fprintf('  [mass table] MTOM = %.3f kg (source of truth) vs iterated params.performance.MTOM = %.3f kg (%.1f%% difference)\n', ...
    MTOM, params.performance.MTOM, 100*(MTOM-params.performance.MTOM)/params.performance.MTOM);

x_LE_wing = params.structures.x_LE_wing;
x_ac_abs  = x_LE_wing + params.geometry.x_ac * c;
x_t_abs   = x_ac_abs + params.geometry.l_t;
x_cg_abs  = sum(massTable.Mass_g .* massTable.X_m) / sum(massTable.Mass_g);

main_mask = strcmp(massTable.Component, 'Main gear (pair)');
if sum(main_mask) ~= 1
    error('TakeOffRotation:massTable', ['MassBudget tab must have exactly one row named ''Main gear (pair)'' ' ...
        '(found %d) -- the rotation moment balance needs it by name.'], sum(main_mask));
end
x_main_gear = massTable.X_m(main_mask);

motor_mask = strcmp(massTable.Component, 'Motor');
if sum(motor_mask) ~= 1
    error('TakeOffRotation:massTable', ['MassBudget tab must have exactly one row named ''Motor'' ' ...
        '(found %d) -- the thrust-line height needs it by name.'], sum(motor_mask));
end

% Heights: the mass table's Height_m column is z up from the fuselage
% underside (same datum convention as the A8 example CG table). Ground
% clearance from the fuselage underside down to the wheel contact patch
% is approximated by the main-gear leg length -- an ASSUMPTION that
% ignores wheel radius, flagged here rather than silently baked in.
ground_clearance = params.geometry.main_gear_len; % ASSUMPTION
z_cg_abs = sum(massTable.Mass_g .* massTable.Height_m) / sum(massTable.Mass_g);
h_cg = ground_clearance + z_cg_abs;               % CG height above the ground
h_T  = ground_clearance + massTable.Height_m(motor_mask); % thrust-line height above the ground

%% Tail elevator authority at full deflection (same model as TailLoads.m)

S_h = params.geometry.S_hstab;
eta_h = params.structures.eta_h;
CL_max_tail = params.structures.CL_max_tail;
CL_delta_e_Tail = params.aero.CL_Alpha_Tail * params.aero.tau_e;
delta_e_max = deg2rad(params.geometry.delta_e_limit_deg);

Lt_raw_fn   = @(V) 0.5*rho*V.^2*S_h*CL_delta_e_Tail*delta_e_max; % elevator-only download, full deflection
Lt_stall_fn = @(V) 0.5*rho*V.^2*S_h*eta_h*CL_max_tail;
Lt_rot_fn   = @(V) min(Lt_raw_fn(V), Lt_stall_fn(V));            % capped by tail stall

%% Ground-roll aerodynamics and representative thrust/acceleration

% Reuses the same constant-acceleration ground-roll approximation already
% adopted in PropulsionSizing.m (T_static), self-contained here so this
% script does not depend on PropulsionSizing.m's internal variables.
CL_R = params.aero.CL_R;         % rotation/ground-roll CL at the as-rigged attitude
CD_0 = params.aero.CD_0;
K_wing = params.aero.K_wing;
V_TO = params.performance.V_TO;
S_TO = params.performance.S_TO;
mu_TO = params.env.mu_TO;

L_w_fn = @(V) 0.5*rho*V.^2*S*CL_R;
D_fn   = @(V) 0.5*rho*V.^2*S*(CD_0 + K_wing*CL_R^2);
Mac_fn = @(V) 0.5*rho*V.^2*S*c*params.aero.CM_ac_w; % code convention: negative = nose-down

L_TO_avg = L_w_fn(0.7*V_TO);
D_TO_avg = D_fn(0.7*V_TO);
a_avg = V_TO^2 / (2*S_TO);                                    % constant-acceleration ground roll
T_avg = (MTOW*V_TO^2)/(2*g*S_TO) + D_TO_avg + mu_TO*(MTOW-L_TO_avg); % same form as PropulsionSizing.m's T_static

%% Moment balance about the main-gear contact (the pivot at rotation)

% Sign convention: positive = nose-down (resists rotation), negative =
% nose-up (drives rotation). Weight ahead of the pivot and thrust height
% above it are nose-down; wing lift ahead of the pivot and tail download
% aft of it are nose-up; drag+inertia at CG height is nose-up about a
% ground-level pivot. See the free-body figure in A8.pdf p.5.
torqueNet = @(V, x_m) MTOW*(x_m - x_cg_abs) ...
    - L_w_fn(V)*(x_m - x_ac_abs) ...
    - Mac_fn(V) ...
    - Lt_rot_fn(V)*(x_t_abs - x_m) ...
    + T_avg*h_T ...
    - (D_fn(V) + MTOM*a_avg)*h_cg;

try
    V_R = fzero(@(V) torqueNet(V, x_main_gear), [0.5, 3*V_TO]);
    rotates_before_TO = V_R <= V_TO;
catch
    V_R = NaN;
    rotates_before_TO = false;
    warning('TakeOffRotation:noRoot', 'No rotation speed found in [0.5, %.1f] m/s at the current main-gear position -- check elevator authority and CG/gear geometry.', 3*V_TO);
end

fprintf('\n=== Take-off rotation: moment balance about the main-gear contact ===\n');
fprintf('  x_cg = %.3f m, x_ac = %.3f m, x_t = %.3f m, x_main_gear = %.3f m (all from the nose)\n', x_cg_abs, x_ac_abs, x_t_abs, x_main_gear);
fprintf('  h_cg = %.3f m, h_T = %.3f m above the ground (ground clearance = main-gear leg length = %.3f m, ASSUMPTION)\n', h_cg, h_T, ground_clearance);
fprintf('  ground roll: a_avg = V_TO^2/(2 S_TO) = %.2f m/s^2, T_avg = %.2f N (same form as PropulsionSizing.m T_static)\n', a_avg, T_avg);
fprintf('  CL_R = %.3f (rotation/ground CL, ASSUMPTION set elsewhere), full-elevator tail download at V_TO = %.2f N (%s)\n', ...
    CL_R, Lt_rot_fn(V_TO), local_ternary(Lt_raw_fn(V_TO) > Lt_stall_fn(V_TO), 'stall-capped', 'elevator auth.'));
if rotates_before_TO
    fprintf('  >>> ROTATION SPEED V_R = %.2f m/s (V_TO = %.2f m/s) -- rotates with %.2f m/s of margin\n', V_R, V_TO, V_TO - V_R);
else
    fprintf('  >>> DOES NOT ROTATE before V_TO = %.2f m/s at the current main-gear position (x_m = %.3f m)\n', V_TO, x_main_gear);
end

%% Aft-most main-gear position that still rotates before V_TO

% V_R(x_m) via a direct root-find (not a coarse sweep): as x_m -> x_t_abs
% the tail's own arm to the pivot vanishes, so its whole nose-up
% contribution disappears there -- V_R can climb steeply and a coarse
% sweep can miss the crossing entirely, silently under-reporting the
% limit. A generous inner bracket, with a large fallback (not NaN) on
% failure, keeps the outer root-find well-behaved even where V_R is very
% large or technically unsolvable.
V_R_of_xm = @(x_m) local_solveVR(@(V) torqueNet(V, x_m), V_TO);

x_lo = x_cg_abs + 0.01;
x_hi = x_t_abs - 1e-4; % main gear must stay ahead of the tail; the arm vanishes exactly at x_t

h_lo = V_R_of_xm(x_lo) - V_TO;
h_hi = V_R_of_xm(x_hi) - V_TO;

if h_lo > 0
    x_m_aft_limit = NaN;
    fprintf('  Even just aft of the CG (x_m = %.3f m) the aircraft does not rotate before V_TO -- no aft-most limit to report.\n', x_lo);
elseif h_hi < 0
    x_m_aft_limit = NaN;
    fprintf('  The aircraft rotates before V_TO everywhere up to the tail (checked to x_m = %.3f m) -- no binding aft-most limit within the physical range.\n', x_hi);
else
    x_m_aft_limit = fzero(@(x_m) V_R_of_xm(x_m) - V_TO, [x_lo, x_hi]);
    fprintf('  Aft-most main-gear position that still rotates before V_TO: x_m = %.3f m (current design: x_m = %.3f m, margin = %.3f m)\n', ...
        x_m_aft_limit, x_main_gear, x_m_aft_limit - x_main_gear);
end

% Sweep purely for the visualization in panel 2 -- the reported limit above
% comes from the root-find, not from this coarser curve.
x_m_sweep = linspace(x_lo, x_hi, 300);
V_R_sweep = arrayfun(V_R_of_xm, x_m_sweep);

%% Plotting

blue = [0.00 0.45 0.74]; orange = [0.85 0.33 0.10]; red = [0.80 0.00 0.00];

figure('Name','Take-off Rotation','Color','w','Position',[100 100 1150 500],'WindowStyle','docked');
tiledlayout(1,2,'TileSpacing','loose','Padding','compact');

% Panel 1: nose-up vs nose-down moment magnitude about the main gear, vs V
nexttile; hold on; box on; grid on;
V_plot = linspace(0.5, 1.3*max(V_TO, isfinite(V_R)*V_R + ~isfinite(V_R)*V_TO), 300);
noseDown = MTOW*(x_main_gear - x_cg_abs) - Mac_fn(V_plot) + T_avg*h_T;
noseUp   = L_w_fn(V_plot)*(x_main_gear - x_ac_abs) + Lt_rot_fn(V_plot)*(x_t_abs - x_main_gear) + (D_fn(V_plot) + MTOM*a_avg)*h_cg;
plot(V_plot, noseDown, '-', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', 'nose-down moment (weight, $M_{ac}$, thrust height)');
plot(V_plot, noseUp, '-', 'Color', blue, 'LineWidth', 1.6, 'DisplayName', 'nose-up moment (wing lift, tail download, drag+inertia)');
yl = ylim; % query the actual visible range -- don't anchor labels at y=0, which may sit outside it
xline(V_TO, ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
text(V_TO, yl(1) + 0.04*(yl(2)-yl(1)), sprintf('  $V_{TO}$=%.1f', V_TO), 'Color', [0.35 0.35 0.35], 'FontSize', 9, 'Interpreter', 'latex', 'VerticalAlignment', 'bottom');
if rotates_before_TO
    plot(V_R, MTOW*(x_main_gear-x_cg_abs)-Mac_fn(V_R)+T_avg*h_T, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 9, 'HandleVisibility', 'off');
    text(V_R, yl(1) + 0.11*(yl(2)-yl(1)), sprintf('  $V_R$=%.1f', V_R), 'Color', red, 'FontSize', 9, 'Interpreter', 'latex', 'VerticalAlignment', 'bottom');
end
xlabel('equivalent airspeed V (m/s)'); ylabel('moment about main gear (N$\cdot$m)', 'Interpreter', 'latex');
title(sprintf('Rotation at $x_m$=%.3f m: crossing = $V_R$', x_main_gear), 'Interpreter', 'latex');
legend('Location', 'best', 'Interpreter', 'latex');

% Panel 2: rotation speed vs main-gear position, with V_TO and the aft-most limit
nexttile; hold on; box on; grid on;
plot(x_m_sweep, V_R_sweep, '-', 'Color', blue, 'LineWidth', 1.8, 'DisplayName', '$V_R(x_m)$');
yline(V_TO, '--', 'Color', orange, 'LineWidth', 1.4, 'DisplayName', sprintf('$V_{TO}$=%.1f m/s', V_TO));
plot(x_main_gear, V_R, 'o', 'MarkerFaceColor', [0.20 0.60 0.30], 'MarkerEdgeColor', 'k', 'MarkerSize', 9, 'DisplayName', 'current design');
yl2 = ylim;
if ~isnan(x_m_aft_limit)
    plot(x_m_aft_limit, V_TO, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 9, 'HandleVisibility', 'off');
    text(x_m_aft_limit, V_TO, sprintf('  aft-most, $x_m$=%.3f m', x_m_aft_limit), 'Color', red, 'FontSize', 9, 'Interpreter', 'latex', 'VerticalAlignment', 'bottom');
elseif V_R_sweep(end) < V_TO
    text(mean(xlim), yl2(1) + 0.06*(yl2(2)-yl2(1)), 'rotates before $V_{TO}$ everywhere up to the tail -- no binding aft-most limit', ...
        'Color', red, 'FontSize', 9, 'Interpreter', 'latex', 'HorizontalAlignment', 'center');
else
    text(mean(xlim), yl2(1) + 0.06*(yl2(2)-yl2(1)), 'does not rotate before $V_{TO}$ anywhere ahead of the tail', ...
        'Color', red, 'FontSize', 9, 'Interpreter', 'latex', 'HorizontalAlignment', 'center');
end
xlabel('main-gear station from the nose, $x_m$ (m)', 'Interpreter', 'latex'); ylabel('rotation speed $V_R$ (m/s)', 'Interpreter', 'latex');
title('Aft-most main-gear position that still rotates before $V_{TO}$', 'Interpreter', 'latex');
legend('Location', 'best', 'Interpreter', 'latex');

sgtitle('Take-off Rotation: Moment Balance About the Main-Gear Contact', 'FontWeight', 'bold');

%% Outputs

outputs.TakeOffRotation = struct('V_R', V_R, 'V_TO', V_TO, 'rotates_before_TO', rotates_before_TO, ...
    'x_main_gear', x_main_gear, 'x_m_aft_limit', x_m_aft_limit, ...
    'x_cg_abs', x_cg_abs, 'x_ac_abs', x_ac_abs, 'x_t_abs', x_t_abs, ...
    'h_cg', h_cg, 'h_T', h_T, 'ground_clearance', ground_clearance, ...
    'T_avg', T_avg, 'a_avg', a_avg);

params.performance.V_R = V_R;
params.performance.x_m_aft_limit = x_m_aft_limit;

end

function V_R = local_solveVR(torqueFn, V_TO)
    % Solves torqueFn(V) = 0 for V. On failure (no rotation speed found
    % within a generous bracket), returns a large-but-finite stand-in
    % rather than NaN, so callers doing their own root-find over this
    % function's output stay well-behaved instead of hitting a NaN.
    try
        V_R = fzero(torqueFn, [0.5, 5*V_TO]);
    catch
        V_R = 5*V_TO;
    end
end

function out = local_ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
