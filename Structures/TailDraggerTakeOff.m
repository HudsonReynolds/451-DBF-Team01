function [outputs, params] = TailDraggerTakeOff(params)

%% A8 Deliverable 4 -- Take-off and landing-gear placement, tail-dragger
%
% Lecture 08B (Landing Gear) treats tail-dragger take-off differently
% from tricycle: there is no nose wheel to lift, so there is no VR<=VTO
% rotation check (criteria comparison table, p.14: tricycle "rotation
% speed VR<=VTO" vs tail-dragger "tail lifts early; check propeller
% clearance tail-up"). This script implements the full tail-dragger
% placement-criteria set from that lecture: tip-forward angle (p.11),
% three-point attitude and tail-wheel load (p.12), overturn angle (p.7,
% applied about the main-to-tail axis per p.9), propeller clearance tail
% up (p.14/16), and the tail-lift moment balance (the take-off analog of
% the tricycle's Mup/VR check, p.8, re-derived for the reversed gear and
% elevator sense -- see the comments below).

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

x_LE_wing = params.structures.x_LE_wing;
x_ac_abs  = x_LE_wing + params.geometry.x_ac * c;
x_t_abs   = x_ac_abs + params.geometry.l_t;
x_cg_abs  = sum(massTable.Mass_g .* massTable.X_m) / sum(massTable.Mass_g);

main_mask = strcmp(massTable.Component, 'Main gear (pair)');
if sum(main_mask) ~= 1
    error('TailDraggerTakeOff:massTable', ['MassBudget tab must have exactly one row named ''Main gear (pair)'' ' ...
        '(found %d) -- the placement criteria need it by name.'], sum(main_mask));
end
x_main_gear = massTable.X_m(main_mask);

tailwheel_mask = strcmp(massTable.Component, 'Tail wheel');
if sum(tailwheel_mask) ~= 1
    error('TailDraggerTakeOff:massTable', ['MassBudget tab must have exactly one row named ''Tail wheel'' ' ...
        '(found %d) -- the placement criteria need it by name.'], sum(tailwheel_mask));
end
x_tailwheel = massTable.X_m(tailwheel_mask);

motor_mask = strcmp(massTable.Component, 'Motor');
if sum(motor_mask) ~= 1
    error('TailDraggerTakeOff:massTable', ['MassBudget tab must have exactly one row named ''Motor'' ' ...
        '(found %d) -- the thrust-line height needs it by name.'], sum(motor_mask));
end

main_gear_len = params.geometry.main_gear_len;
tail_gear_len = params.geometry.tail_gear_len;
main_wheel_r  = params.geometry.main_wheel_dia / 2;
tail_wheel_r  = params.geometry.tail_wheel_dia / 2;
gear_track    = params.structures.gear_track; % ASSUMPTION, added for the overturn-angle check

% Heights: the mass table's Z_m column is z up from the fuselage
% underside (same datum convention as the A8 example CG table). Ground
% clearance from the fuselage underside down to the ACTUAL ground contact
% patch is the leg length (attachment point to axle) PLUS the wheel
% radius (axle to ground) -- both terms are real parameters already in
% Sheet1 (main_gear_len/tail_gear_len and main_wheel_dia/tail_wheel_dia),
% just not previously combined here. This "level, tail-up" reference
% frame is what matters for the take-off roll and for propeller clearance
% in the tail-up attitude (the critical one, lecture p.16); the
% three-point (static, nose-up) attitude used for the placement-criteria
% schematic below is a separate, tilted frame and re-derives its own
% ground clearance the same way (leg + wheel radius) for both wheels.
ground_clearance_main = main_gear_len + main_wheel_r;
ground_clearance_tail = tail_gear_len + tail_wheel_r;
z_cg_abs = sum(massTable.Mass_g .* massTable.Z_m) / sum(massTable.Mass_g);
h_cg = ground_clearance_main + z_cg_abs;               % CG height above the ground, level attitude
h_T  = ground_clearance_main + massTable.Z_m(motor_mask); % thrust-line height above the ground, level attitude

%% Tail authority: UPWARD force from full DOWN elevator (lifts the tail)
%
% Opposite sense from the tricycle case: there, up-elevator makes a tail
% DOWNLOAD that lifts the nose. Here, down-elevator makes an UPWARD tail
% force that lifts the tail wheel clear of the ground.

S_h = params.geometry.S_hstab;
eta_h = params.structures.eta_h;
CL_max_tail = params.structures.CL_max_tail;
CL_delta_e_Tail = params.aero.CL_Alpha_Tail * params.aero.tau_e;
delta_e_max = deg2rad(params.geometry.delta_e_limit_deg);

Lt_raw_fn   = @(V) 0.5*rho*V.^2*S_h*CL_delta_e_Tail*delta_e_max; % elevator-only upward force, full deflection
Lt_stall_fn = @(V) 0.5*rho*V.^2*S_h*eta_h*CL_max_tail;
Lt_up_fn    = @(V) min(Lt_raw_fn(V), Lt_stall_fn(V));            % capped by tail stall

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

%% Moment balance about the main-gear contact (the pivot; now forward, near the CG)
%
% Sign convention: positive = tail lifts. Weight is now AFT of the pivot
% (CG behind the mains, the tail-dragger static-stability requirement)
% and holds the tail down; wing lift and the up-elevator tail force both
% help lift the tail; Mac and drag+inertia oppose it the same way they
% did in the tricycle formula (their sense doesn't depend on which way
% the gear points, only the pivot's height, which is unchanged). Cross-
% checked term-by-term against lecture 08B p.8's Mup formula: every term
% here matches that formula's structure except the tail term, which
% flips sign because the elevator now pushes the tail up, not down.
torqueTailUp = @(V, x_m) MTOW*(x_m - x_cg_abs) ...
    - L_w_fn(V)*(x_m - x_ac_abs) ...
    - Mac_fn(V) ...
    + Lt_up_fn(V)*(x_tailwheel - x_m) ...
    + T_avg*h_T ...
    - (D_fn(V) + MTOM*a_avg)*h_cg;

try
    V_tailUp = fzero(@(V) torqueTailUp(V, x_main_gear), [0.1, 3*V_TO]);
    lifts_before_TO = V_tailUp <= V_TO;
catch
    V_tailUp = NaN;
    lifts_before_TO = false;
    warning('TailDraggerTakeOff:noRoot', 'No tail-lift speed found in [0.1, %.1f] m/s -- check elevator authority and CG/gear geometry.', 3*V_TO);
end

%% Placement criteria (lecture 08B, tail-dragger column of the p.14 table)

% Tip-forward angle, p.11: alpha_tf = atan((xcg-xm)/hcg), required 16-25 deg.
alpha_tf_deg = atand((x_cg_abs - x_main_gear) / h_cg);
tf_ok = alpha_tf_deg >= 16 && alpha_tf_deg <= 25;

% Tail-wheel static load, p.8: Ft/W = (xcg-xm)/(xt-xm).
Ft_W = (x_cg_abs - x_main_gear) / (x_tailwheel - x_main_gear);
ft_ok = Ft_W >= 0.03 && Ft_W <= 0.15; % lecture: "a few per cent to about 15%"

% Three-point attitude, p.12: theta3 = atan((leg_main-leg_tail)/(xt-xm)),
% aircraft resting on all three wheels simultaneously (nose-up, since the
% main gear's ground clearance is taller than the tail wheel's). Uses
% full ground clearance (leg + wheel radius) at each end, not just the
% bare leg length. alpha3 = theta3 + iw must clear the stall angle by 2
% deg. Wing incidence iw is taken as zero to the body datum, the SAME
% assumption the lecture's own example makes ("not given for the example"
% -- p.20); alpha_stall is estimated the same first-order way as the
% lecture's own three-point check, from the linear lift model:
% alpha_stall = alpha_0L + CL_max/CL_alpha.
theta3_deg = atand((ground_clearance_main - ground_clearance_tail) / (x_tailwheel - x_main_gear));
i_w_deg = 0; % ASSUMPTION, matching lecture 08B's own worked example
alpha3_deg = theta3_deg + i_w_deg;
alpha_0L_deg = -rad2deg(params.aero.CL_0 / params.aero.CL_Alpha);
alpha_stall_deg = alpha_0L_deg + rad2deg(params.aero.CL_max / params.aero.CL_Alpha);
three_pt_ok = alpha3_deg <= alpha_stall_deg - 2;

% Exposed for A9's TakeoffPerformance.m: the lift coefficient the aircraft
% actually sits at in this three-point ground attitude (linear lift model,
% same CL = CL_Alpha*(alpha-alpha_0L) used for alpha_stall_deg above) --
% "CLg from the ground attitude," not an arbitrary assumed value. Only
% meaningful once this script has run (needs the as-placed landing-gear
% geometry), so A9 falls back to its own placeholder if this hasn't run.
CL_ground_attitude = params.aero.CL_Alpha * deg2rad(alpha3_deg - alpha_0L_deg);
params.aero.CL_ground_attitude = CL_ground_attitude;

% Overturn angle, p.7/9: about the main-to-tail axis for a tail-dragger,
% d = plan distance from the CG to that axis, phi = atan(hcg/d) <= 63 deg.
% Needs a lateral track width, not previously modelled anywhere in this
% project (every other A8 script is 2-D, x-z only) -- added as
% params.structures.gear_track, ASSUMPTION.
pa = [x_main_gear, gear_track/2]; % one main wheel, plan view
pb = [x_tailwheel, 0];            % tail wheel, assumed on the centreline
pcg = [x_cg_abs, 0];
d_overturn = abs((pb(1)-pa(1))*(pcg(2)-pa(2)) - (pb(2)-pa(2))*(pcg(1)-pa(1))) / norm(pb-pa);
phi_deg = atand(h_cg / d_overturn);
overturn_ok = phi_deg <= 63;

%% Propeller clearance, tail up (lecture 08B p.14/16): the criterion that
%% actually replaces the tricycle VR<=VTO check for a tail-dragger.

IN2M = 0.0254;
D_prop = IN2M * str2double(regexp(params.prop.prop_file, '_(\d+)x', 'tokens', 'once')); % same parse as PropulsionSizing.m
R_prop = D_prop / 2;
prop_clearance = h_T - R_prop;
prop_ok = prop_clearance >= 0.025; % lecture example's own reference floor, >=25 mm

fprintf('\n--- Landing Gear Placement (Deliverable 4) ---\n');
fprintf('  Tail-lift speed:           %.1f m/s vs V_TO = %.1f m/s          -- %s\n', V_tailUp, V_TO, local_ternary(lifts_before_TO,'PASS','FAIL'));
fprintf('  Tip-forward angle:         %.1f deg  [16 - 25 deg]              -- %s\n', alpha_tf_deg, local_ternary(tf_ok,'PASS','FAIL'));
fprintf('  Tail-wheel static load:    %.1f %%  [~3 - 15%%]                  -- %s\n', 100*Ft_W, local_ternary(ft_ok,'PASS','FAIL'));
fprintf('  Three-point attitude:      %.1f deg vs stall-2 = %.1f deg       -- %s\n', alpha3_deg, alpha_stall_deg-2, local_ternary(three_pt_ok,'PASS','FAIL'));
fprintf('  Overturn angle:            %.1f deg  [<= 63 deg]                -- %s\n', phi_deg, local_ternary(overturn_ok,'PASS','FAIL'));
fprintf('  Prop. clearance (tail up): %.0f mm  [>= 25 mm]                  -- %s\n', prop_clearance*1000, local_ternary(prop_ok,'PASS','FAIL'));

%% Plotting

blue = [0.00 0.45 0.74]; orange = [0.85 0.33 0.10]; red = [0.80 0.00 0.00]; green = [0.20 0.60 0.30];

figure('Name','Landing Gear Placement','Color','w','WindowStyle','docked');
tiledlayout(2,2,'TileSpacing','loose','Padding','loose');

% Panel 1: side-view schematic, three-point (static) attitude -- matches
% lecture 08B p.26 style: ground frame, legs, underside reference line,
% CG, tip-forward angle.
nexttile; hold on; box on; grid on;
% Ground clearance at each end = leg length (attachment to axle) + wheel
% radius (axle to ground) -- both wheels drawn to scale, axle at wheel_r
% above the ground, so the underside reference line spans attachment
% points, not axle heights.
slope3pt = (ground_clearance_tail - ground_clearance_main) / (x_tailwheel - x_main_gear);
underside_at = @(x) ground_clearance_main + slope3pt*(x - x_main_gear);
prop_x_offset = R_prop*sin(deg2rad(theta3_deg)); % how far the tilted propeller (below) reaches ahead of x=0
x_nose_plot = min(-0.05, -prop_x_offset - 0.02);
x_aft_plot  = x_tailwheel*1.08;
plot([x_nose_plot, x_aft_plot], [0 0], 'k-', 'LineWidth', 1.2, 'HandleVisibility', 'off');
plot([x_nose_plot, x_aft_plot], underside_at([x_nose_plot, x_aft_plot]), '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.3, 'DisplayName', 'fuselage underside, 3-pt attitude');
plot([x_main_gear x_main_gear], [main_wheel_r ground_clearance_main], '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 2.2, 'DisplayName', 'main gear leg');
plot([x_tailwheel x_tailwheel], [tail_wheel_r ground_clearance_tail], '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.6, 'HandleVisibility', 'off');
th = linspace(0, 2*pi, 60);
plot(x_main_gear + main_wheel_r*cos(th), main_wheel_r + main_wheel_r*sin(th), '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.4, 'DisplayName', 'wheels (to scale)');
plot(x_tailwheel + tail_wheel_r*cos(th), tail_wheel_r + tail_wheel_r*sin(th), '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.2, 'HandleVisibility', 'off');
z_cg_3pt = underside_at(x_cg_abs) + z_cg_abs;
plot(x_cg_abs, z_cg_3pt, 'o', 'MarkerSize', 9, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w', 'LineWidth', 1.5, 'DisplayName', 'CG');
plot([x_cg_abs, x_main_gear], [z_cg_3pt, 0], ':', 'Color', red, 'LineWidth', 1.4, 'DisplayName', sprintf('tip-forward, %.1f deg', alpha_tf_deg));
% Propeller, to scale like the wheels above: seen from this side view the
% disc is edge-on, so it draws as a line spanning its full diameter
% (D_prop, R_prop already computed below in the propeller-clearance
% section), centred on the thrust line -- NOT drawn plumb-vertical, since
% the disc is mounted perpendicular to the thrust line/fuselage reference
% line, which is itself tilted by theta3 (nose-up) in this attitude. The
% fuselage line's direction is (cos(theta3), -sin(theta3)) (it descends
% going aft, per underside_at's slope), so the perpendicular -- the
% propeller's own line -- is (sin(theta3), cos(theta3)): at theta3=0 this
% reduces to plumb-vertical, as expected.
theta3_rad = deg2rad(theta3_deg);
prop_dir = [sin(theta3_rad), cos(theta3_rad)];
z_motor_3pt = underside_at(0) + massTable.Z_m(motor_mask);
prop_hub = [0, z_motor_3pt];
prop_tip1 = prop_hub + R_prop*prop_dir;
prop_tip2 = prop_hub - R_prop*prop_dir;
plot([prop_tip2(1) prop_tip1(1)], [prop_tip2(2) prop_tip1(2)], '-', 'Color', orange, 'LineWidth', 2.5, ...
    'DisplayName', sprintf('propeller, %.0f in dia. (3-pt attitude)', D_prop/IN2M));
plot(prop_hub(1), prop_hub(2), 'o', 'MarkerSize', 4, 'MarkerFaceColor', orange, 'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
text(x_main_gear, -0.03, sprintf('main, x=%.2f m', x_main_gear), 'FontSize', 8, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
text(x_tailwheel, -0.03, sprintf('tail wheel, x=%.2f m', x_tailwheel), 'FontSize', 8, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
text(mean([x_main_gear x_tailwheel]), 0.5*(ground_clearance_main+ground_clearance_tail)*0.5, sprintf('$\\theta_3$=%.1f deg', theta3_deg), ...
    'FontSize', 9, 'Interpreter', 'latex', 'Color', [0.4 0.4 0.4]);
xlabel('station from the nose, x (m)'); ylabel('height above ground (m)');
title('Three-point attitude and tip-forward angle (lecture 08B p.11-12 style)');
legend('Location', 'best', 'FontSize', 7);
xlim([x_nose_plot, x_aft_plot]);
ylim([-0.05, max([z_cg_3pt, ground_clearance_main, z_motor_3pt + R_prop])*1.6]);

% Panel 2: criteria summary, matching lecture 08B p.27's table.
nexttile; axis off;
rows = {
    'Tip-forward angle',      sprintf('%.1f deg', alpha_tf_deg),  '16 - 25 deg', tf_ok
    'Tail-wheel static load', sprintf('%.1f %%', 100*Ft_W),       '~3 - 15 %',   ft_ok
    'Overturn angle',         sprintf('%.1f deg', phi_deg),       '<= 63 deg',   overturn_ok
    'Three-point attitude',   sprintf('%.1f deg', alpha3_deg),    sprintf('<= %.1f deg', alpha_stall_deg-2), three_pt_ok
    'Prop. clearance (tail up)', sprintf('%.0f mm', prop_clearance*1000), '>= 25 mm', prop_ok
    'Tail-lift speed',        sprintf('%.1f m/s', V_tailUp),      sprintf('<= %.1f m/s', V_TO), lifts_before_TO
    };
% Right-aligned numeric columns: immune to the criterion label's length,
% since each column is anchored by its own right edge, not its left.
col_val = 0.62; col_req = 0.90; col_verdict = 1.05;
y0 = 0.95; dy = 0.13;
% Interpreter 'none' throughout this table -- it's plain text, not math,
% and several cells contain a literal '%' (e.g. "9.2 %"), which the
% default LaTeX interpreter would otherwise try (and fail) to parse as a
% comment character.
text(0, y0+0.08, 'Placement criteria (lecture 08B, tail-dragger)', 'FontWeight', 'bold', 'FontSize', 11, 'Interpreter', 'none');
text(0, y0-0.03, 'criterion', 'FontWeight', 'bold', 'FontSize', 9, 'Interpreter', 'none');
text(col_val, y0-0.03, 'value', 'FontWeight', 'bold', 'FontSize', 9, 'HorizontalAlignment', 'right', 'Interpreter', 'none');
text(col_req, y0-0.03, 'req.', 'FontWeight', 'bold', 'FontSize', 9, 'HorizontalAlignment', 'right', 'Interpreter', 'none');
text(col_verdict, y0-0.03, 'verdict', 'FontWeight', 'bold', 'FontSize', 9, 'HorizontalAlignment', 'right', 'Interpreter', 'none');
for i = 1:size(rows,1)
    yi = y0 - 0.03 - dy*i;
    verdictColor = local_ternary(rows{i,4}, green, red);
    verdictText  = local_ternary(rows{i,4}, 'PASS', 'FAIL');
    text(0, yi, rows{i,1}, 'FontSize', 8.5, 'Interpreter', 'none');
    text(col_val, yi, rows{i,2}, 'FontSize', 8.5, 'HorizontalAlignment', 'right', 'Interpreter', 'none');
    text(col_req, yi, rows{i,3}, 'FontSize', 8.5, 'HorizontalAlignment', 'right', 'Interpreter', 'none');
    text(col_verdict, yi, verdictText, 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', verdictColor, 'HorizontalAlignment', 'right', 'Interpreter', 'none');
end
xlim([0 1.15]); ylim([0 1]);

% Panel 3: tail-lift moment crossing plot.
nexttile; hold on; box on; grid on;
V_plot = linspace(0.5, 1.3*max(V_TO, isfinite(V_tailUp)*V_tailUp + ~isfinite(V_tailUp)*V_TO), 300);
tailDown = MTOW*(x_cg_abs - x_main_gear) + Mac_fn(V_plot) + (D_fn(V_plot) + MTOM*a_avg)*h_cg;
tailUp   = L_w_fn(V_plot)*(x_main_gear - x_ac_abs) + Lt_up_fn(V_plot)*(x_tailwheel - x_main_gear) + T_avg*h_T;
plot(V_plot, tailDown, '-', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', 'tail-down moment (weight, $M_{ac}$, drag+inertia)');
plot(V_plot, tailUp, '-', 'Color', blue, 'LineWidth', 1.6, 'DisplayName', 'tail-up moment (wing lift, elevator, thrust height)');
yl = ylim;
xline(V_TO, ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
text(V_TO, yl(1) + 0.04*(yl(2)-yl(1)), sprintf('  $V_{TO}$=%.1f', V_TO), 'Color', [0.35 0.35 0.35], 'FontSize', 9, 'Interpreter', 'latex', 'VerticalAlignment', 'bottom');
if lifts_before_TO
    plot(V_tailUp, MTOW*(x_cg_abs-x_main_gear)+Mac_fn(V_tailUp)+(D_fn(V_tailUp)+MTOM*a_avg)*h_cg, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 9, 'HandleVisibility', 'off');
    text(V_tailUp, yl(1) + 0.11*(yl(2)-yl(1)), sprintf('  $V_{tail-up}$=%.1f', V_tailUp), 'Color', red, 'FontSize', 9, 'Interpreter', 'latex', 'VerticalAlignment', 'bottom');
end
xlabel('equivalent airspeed V (m/s)'); ylabel('moment about main gear (N$\cdot$m)', 'Interpreter', 'latex');
title(sprintf('Tail lift at $x_m$=%.3f m: crossing = tail-lift speed', x_main_gear), 'Interpreter', 'latex');
legend('Location', 'best', 'Interpreter', 'latex', 'FontSize', 8);

% Panel 4: propeller clearance vs main-gear leg length -- the actual
% governing design sensitivity for a tail-dragger (lecture 08B p.14/16).
% Sweeps the leg (the easily-adjustable design variable) with the wheel
% radius held fixed at its current value, consistent with h_T above.
nexttile; hold on; box on; grid on;
leg_sweep = linspace(0.5*main_gear_len, 1.8*main_gear_len, 200);
h_T_sweep = leg_sweep + main_wheel_r + massTable.Z_m(motor_mask);
clearance_sweep = h_T_sweep - R_prop;
plot(leg_sweep*1000, clearance_sweep*1000, '-', 'Color', blue, 'LineWidth', 1.8, 'DisplayName', 'clearance$(\ell_{main})$');
yline(0, 'k-', 'HandleVisibility', 'off');
yline(25, '--', 'Color', orange, 'LineWidth', 1.2, 'DisplayName', '25 mm reference (lecture example)');
plot(main_gear_len*1000, prop_clearance*1000, 'o', 'MarkerFaceColor', green, 'MarkerEdgeColor', 'k', 'MarkerSize', 9, 'DisplayName', 'current assumption');
xlabel(sprintf('main-gear leg length, $\\ell_{main}$ (mm), wheel radius fixed at %.0f mm', main_wheel_r*1000), 'Interpreter', 'latex'); ylabel('propeller clearance, tail up (mm)', 'Interpreter', 'latex');
title('Propeller clearance vs. main-gear leg length', 'Interpreter', 'latex');
legend('Location', 'best', 'Interpreter', 'latex', 'FontSize', 8);

sgtitle('Deliverable 4: Take-off and Landing-Gear Placement, Tail-Dragger Configuration', 'FontWeight', 'bold');

%% Outputs

outputs.TailDraggerTakeOff = struct('V_tailUp', V_tailUp, 'V_TO', V_TO, 'lifts_before_TO', lifts_before_TO, ...
    'x_main_gear', x_main_gear, 'x_tailwheel', x_tailwheel, ...
    'x_cg_abs', x_cg_abs, 'x_ac_abs', x_ac_abs, 'x_t_abs', x_t_abs, ...
    'h_cg', h_cg, 'h_T', h_T, 'ground_clearance_main', ground_clearance_main, 'ground_clearance_tail', ground_clearance_tail, ...
    'main_wheel_r', main_wheel_r, 'tail_wheel_r', tail_wheel_r, ...
    'alpha_tf_deg', alpha_tf_deg, 'tip_forward_ok', tf_ok, ...
    'Ft_W', Ft_W, 'tail_wheel_load_ok', ft_ok, ...
    'theta3_deg', theta3_deg, 'alpha3_deg', alpha3_deg, 'alpha_stall_deg', alpha_stall_deg, 'three_point_ok', three_pt_ok, ...
    'CL_ground_attitude', CL_ground_attitude, ...
    'phi_overturn_deg', phi_deg, 'overturn_ok', overturn_ok, 'gear_track', gear_track, ...
    'prop_diameter', D_prop, 'prop_clearance', prop_clearance, 'prop_clearance_ok', prop_ok, ...
    'T_avg', T_avg, 'a_avg', a_avg);

params.performance.V_tailUp = V_tailUp;
params.structures.alpha_tf_deg = alpha_tf_deg;
params.structures.prop_clearance_tailUp = prop_clearance;
params.structures.overturn_phi_deg = phi_deg;

end

function out = local_ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
