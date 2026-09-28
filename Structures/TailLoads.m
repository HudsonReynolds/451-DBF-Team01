function [outputs, params] = TailLoads(params)

% A8 Deliverable 3 -- Tail (lecture "08 - Structures", slides 22-26).
% Same method as the wing: spanwise load, integrate to shear and bending;
% new here is the torsion the fin's asymmetric side load puts on the
% tailcone.
%
% Symmetric (horizontal tail, slide 23): balancing load (pitch equilibrium
% at n+, slide 30's formula) plus the elevator increment, capped by the
% tail's stall, evaluated at the worst of V_A, V_C and V_D.
%
% Asymmetric (fin/rudder, slide 24): full rudder at V_D -- fin side force
% capped by fin stall, elliptical spanwise load on the fin sized to the
% root moment, and the torsion + lateral bending that side force puts on
% the tailcone. This deliverable stops at LOADS (matching A8's bullet
% list); stress and margin against an installed section is Deliverable 5.
%
% eta_h, eta_v (tail efficiency), CL_max_tail and D_fus_tail (tailboom
% diameter at the fin) are Sheet1 "structures" ASSUMPTIONs. None of this
% codebase's other stability/trim analysis models tail efficiency, so
% eta_h = eta_v = 1 keeps this consistent with the rest of the aircraft
% model rather than introducing an unmatched wake-reduction factor.
% l_tc (tailcone moment arm for lateral bending) is approximated as l_t
% (the wing-to-tail arm) -- no separate tailcone-length parameter exists
% anywhere in this codebase.
% The tail's own spar chordwise location and Cm,ac are not modelled
% separately: the torque calc reuses the wing's x_spar fraction and
% x_ac=0.25 (thin-airfoil theory places the AC near quarter-chord for any
% thin section, symmetric or not), and sets Cm,ac,h = 0 since the NACA
% 0010 tail section is symmetric (zero pitching moment about its own AC).

rho  = params.env.rho;
MTOW = params.performance.MTOW;
S    = params.geometry.S_wing;
c    = params.geometry.c_wing;

n_pos = params.performance.n_limit_pos;
V_A = params.performance.V_A;
V_C = params.performance.V_C;
V_D = params.performance.V_D;

eta_h = params.structures.eta_h;
eta_v = params.structures.eta_v;
CL_max_tail = params.structures.CL_max_tail;

blue = [0.00 0.45 0.74]; orange = [0.85 0.33 0.10]; teal = [0.00 0.60 0.50];

%% ================= HORIZONTAL TAIL (symmetric, full elevator) =================
S_h = params.geometry.S_hstab;
c_h = params.geometry.c_hstab;
b_h = params.geometry.span_hstab;
s_h = b_h/2;

x_ac = params.geometry.x_ac;
x_cg = params.geometry.x_cg_design;
l_t  = params.geometry.l_t;
CM_ac_w = params.aero.CM_ac_w; % wing's own section/AC moment -- a pure couple, NOT the aircraft-total CM_0
CL_delta_e_Tail = params.aero.CL_Alpha_Tail * params.aero.tau_e;
delta_e_max = deg2rad(params.geometry.delta_e_limit_deg);

% Balancing load (pitch equilibrium at n+, slide 30) plus the elevator
% increment (slide 23), capped by the tail's own stall. Mac uses CM_ac_w,
% not the aircraft-total CM_0: the moment equation already carries the
% lift-arm effect explicitly via the n*MTOW*(x_cg-x_ac) term, so folding
% in CM_0 (which itself already includes a CL_0*(x_cg-x_ac) contribution)
% would double-count that arm.
L_balance   = @(V) (n_pos*MTOW*(x_cg-x_ac)*c + 0.5*rho*V.^2*S*c*CM_ac_w) / l_t;
dL_elev     = @(V) 0.5*rho*V.^2*S_h*CL_delta_e_Tail*delta_e_max;
L_stall_h   = @(V) 0.5*rho*V.^2*S_h*eta_h*CL_max_tail;
L_raw_h     = @(V) L_balance(V) + dL_elev(V);
L_design_h  = @(V) sign(L_raw_h(V)) .* min(abs(L_raw_h(V)), L_stall_h(V));

V_sweep_h = linspace(10, V_D, 200); % starts at 10 m/s to match the lecture's reference plot (slide 23)
L_h_curve = L_design_h(V_sweep_h);
L_h_stall_curve = L_stall_h(V_sweep_h);

V_points = [V_A, V_C, V_D];
point_labels = {'V_A', 'V_C', 'V_D'};
L_raw_points    = L_raw_h(V_points);
L_stall_points  = L_stall_h(V_points);
L_design_points = sign(L_raw_points) .* min(abs(L_raw_points), L_stall_points);
capped_points   = abs(L_raw_points) > L_stall_points;

[L_h_design, i_gov_h] = max(abs(L_design_points));
V_h_governing = V_points(i_gov_h);
L_h_design_signed = L_design_points(i_gov_h);

fprintf('=== Horizontal tail: symmetric case (full elevator, %.0f deg) ===\n', params.geometry.delta_e_limit_deg);
for i = 1:3
    fprintf('  at %s = %.1f m/s: balance+elevator = %+.2f N, stall cap = %.2f N -> design = %+.2f N (%s)\n', ...
        point_labels{i}, V_points(i), L_raw_points(i), L_stall_points(i), L_design_points(i), ...
        local_ternary(capped_points(i), 'capped by tail stall', 'uncapped, elevator authority governs'));
end
fprintf('  >>> GOVERNS: %s = %.1f m/s, design load = %+.2f N\n', point_labels{i_gov_h}, V_h_governing, L_h_design_signed);

% Shrenk approximation on the horizontal tail, same method as the wing:
y_h = linspace(0, s_h, 300);
c_ell_h = (4*S_h/(pi*b_h)) * sqrt(max(1 - (2*y_h/b_h).^2, 0));
c_S_h = 0.5*(c_h + c_ell_h);
IcS_h = trapz(y_h, c_S_h);
k_h = (abs(L_h_design_signed)/2) / IcS_h; % STATE this normalising factor
w_h = sign(L_h_design_signed) * k_h * c_S_h;
[V_h, M_h] = local_tipIntegrate(y_h, w_h);

% Torque (bonus, same method as the wing -- see header note on the
% assumptions this reuses):
e_arm_h = (params.structures.x_spar - x_ac) * c_h;
t_h = w_h*e_arm_h; % + q*c^2*Cm_ac,h, but Cm_ac,h = 0 for a symmetric section
T_h = local_tipIntegrate(y_h, t_h);

fprintf('  Shrenk normalising factor k_h = %.2f N/m^2 (semispan integral = |L_h|/2 = %.2f N)\n', k_h, abs(L_h_design_signed)/2);
fprintf('  root: M(0) = %+.3f N*m, T(0) = %+.4f N*m; BCs at tip: M(s)=%.2e N*m, T(s)=%.2e N*m (should be ~0)\n', ...
    M_h(1), T_h(1), M_h(end), T_h(end));

%% ================= VERTICAL TAIL / FIN (asymmetric, full rudder at V_D) =================
S_v = params.geometry.S_fin; % single-fin area (n_vfins = 1)
H   = params.geometry.span_vstab;
av  = params.aero.CL_Alpha_VT;
tau_r = params.aero.tau_r;
delta_r_max = deg2rad(params.geometry.delta_r_limit);
D_fus = params.structures.D_fus_tail;
l_tc  = params.geometry.l_t; % APPROXIMATION -- no separate tailcone-length parameter; see header note

Yv_linear_fn = @(V) 0.5*rho*V.^2*S_v*eta_v*av*tau_r*delta_r_max;
Yv_stall_fn  = @(V) 0.5*rho*V.^2*S_v*eta_v*CL_max_tail;

V_sweep_v = linspace(10, V_D, 200); % starts at 10 m/s to match the lecture's reference plot (slide 23-style)
Yv_curve = min(Yv_linear_fn(V_sweep_v), Yv_stall_fn(V_sweep_v));
Yv_stall_curve = Yv_stall_fn(V_sweep_v);

Yv_linear = Yv_linear_fn(V_D);
Yv_stall  = Yv_stall_fn(V_D);
Yv = min(Yv_linear, Yv_stall);
capped_rudder = Yv_linear > Yv_stall;

z = linspace(0, H, 300);
w_fin = (4*Yv/(pi*H)) * sqrt(max(1 - (z/H).^2, 0));
[V_fin, M_fin] = local_tipIntegrate(z, w_fin);
M_fin_closed_form = 4*Yv*H/(3*pi); % closed-form cross-check of the elliptical integral

z_cp = D_fus/2 + 4*H/(3*pi);
T_tailcone   = Yv*z_cp;   % concentrated torque, applied at the fin end
M_l_tailcone = Yv*l_tc;   % bending from that same force, at the fuselage (root) end

% Yv acts as a single concentrated load at the fin end of the tailcone,
% with nothing else loading the tailcone along its length. That makes
% torsion T a CONSTANT along the whole length (same reason shear is
% constant between a point load and its support), while the lateral
% bending M_l is NOT a single number -- it ramps linearly from Yv*l_tc at
% the fuselage root down to zero at the fin end. Built as a proper
% distribution here rather than two disconnected scalars, using the same
% root-to-tip convention as the wing and horizontal tail (x=0 at the
% root, where bending peaks; x increases out to the free/fin end, where
% it goes to zero):
x_tc = linspace(0, l_tc, 200); % x=0 at the fuselage root, x=l_tc at the fin end
T_tc   = T_tailcone * ones(size(x_tc));
M_l_tc = Yv * (l_tc - x_tc);

fprintf('\n=== Vertical tail / fin: asymmetric case (full rudder, %.0f deg, at V_D) ===\n', params.geometry.delta_r_limit);
fprintf('  evaluated at V_D = %.1f m/s -- the tail''s stated design point (dive speed, slides 9 and 22)\n', V_D);
fprintf('  Yv: linear = %.2f N, stall cap = %.2f N -> Yv = %.2f N (%s)\n', Yv_linear, Yv_stall, Yv, ...
    local_ternary(capped_rudder, 'capped by fin stall', 'uncapped, rudder authority governs'));
fprintf('  fin root bending M(0) = %.3f N*m (closed-form 4 Yv H/(3 pi) = %.3f N*m, cross-check)\n', M_fin(1), M_fin_closed_form);
fprintf('  z_cp = D_fus/2 + 4H/(3 pi) = %.4f m; torsion into the tailcone T = Yv*z_cp = %.3f N*m (constant along the tailcone)\n', z_cp, T_tailcone);
fprintf('  lateral bending into the tailcone: M_l(root)=%.3f N*m at the fuselage, decreasing to 0 at the fin end (l_tc ~ l_t = %.3f m, APPROXIMATION)\n', M_l_tailcone, l_tc);

%% ---- Figure: horizontal tail (symmetric) ----
% Two panels, matching the lecture's own layout for this case: the design
% load against airspeed (left), and running load + bending + torque
% together on one panel with a dual axis (right) -- w is 1-2 orders of
% magnitude bigger than M and T here, the same reason the fin got split
% into separate panels, but the lecture's own reference figure for this
% specific case uses a shared dual-axis panel, so this matches that.
figure('Name','Tail Loads - Horizontal Tail (Symmetric)','Color','w','Position',[100 100 1150 550],'WindowStyle','docked');
tiledlayout(1,2,'TileSpacing','loose','Padding','compact');

nexttile; hold on; box on; grid on;
plot(V_sweep_h, abs(L_h_curve), '-', 'Color', blue, 'LineWidth', 1.6, 'DisplayName', 'design load, |L_h|');
plot(V_sweep_h, L_h_stall_curve, ':', 'Color', orange, 'LineWidth', 1.4, 'DisplayName', 'stall cap, $q S_h \eta_h C_{L,max}$');
yline(MTOW, '-.', sprintf('aircraft weight %.1f N', MTOW), 'Color', [0.20 0.60 0.30], 'LineWidth', 1.2, ...
    'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
plot(V_h_governing, L_h_design, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7, 'HandleVisibility', 'off');
yl = ylim; row_y_h = yl(1) + [0.03, 0.10]*(yl(2)-yl(1));
for i = 1:3
    xline(V_points(i), ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
    text(V_points(i), row_y_h(mod(i-1,2)+1), sprintf('$%s=%.1f$', point_labels{i}, V_points(i)), ...
        'HorizontalAlignment', 'center', 'Color', [0.35 0.35 0.35], 'FontSize', 9, 'Interpreter', 'latex');
end
xlabel('equivalent airspeed V (m/s)'); ylabel('horizontal tail load |L_h| (N)');
title(sprintf('Horizontal tail load: %.1f N at $%s$ (%s)', L_h_design, point_labels{i_gov_h}, ...
    local_ternary(capped_points(i_gov_h),'stall-capped','elevator auth.')), 'Interpreter', 'latex');
legend('Location', 'best', 'Interpreter', 'latex');

nexttile; hold on; box on; grid on;
yyaxis left
plot(y_h, w_h, '-', 'LineWidth', 1.6, 'DisplayName', 'running load w');
ylabel('running load w (N/m)');
yyaxis right
plot(y_h, M_h, '-', 'LineWidth', 1.6, 'DisplayName', 'bending M');
plot(y_h, T_h, '--', 'LineWidth', 1.6, 'DisplayName', 'torque T');
ylabel('M, T (N$\cdot$m)', 'Interpreter', 'latex');
ylim([0, max([M_h, T_h])*1.1]); % clip to 0 -- neither M nor T goes negative, don't let autoscale pad below it
xlabel('station from centreline y (m)');
title(sprintf('Root M=%.2f, T=%.3f N*m (semispan %.0f mm)', M_h(1), T_h(1), s_h*1000));
legend('Location', 'best', 'Interpreter', 'latex');

sgtitle(sprintf('Symmetric case: full elevator, design point at $%s$; ultimate 1.5$\\times$', point_labels{i_gov_h}), ...
    'Interpreter', 'latex', 'FontWeight', 'bold');

%% ---- Figure: vertical tail / fin (asymmetric) ----
% Two load panels, formatted the same way as the symmetric-case figure:
% running load + bending on one dual-axis panel (fin), and torsion +
% lateral bending on one shared-axis panel (tailcone) -- T and M_l are
% both N*m and only ~5x apart, close enough for one axis to stay readable
% without a dual axis.
figure('Name','Tail Loads - Vertical Tail (Asymmetric Rudder)','Color','w','Position',[1200 100 1150 650],'WindowStyle','docked');
tiledlayout(2,2,'TileSpacing','loose','Padding','compact');

nexttile; hold on; box on; grid on;
plot(V_sweep_v, Yv_curve, '-', 'Color', teal, 'LineWidth', 1.6, 'DisplayName', 'design load, $Y_v$');
plot(V_sweep_v, Yv_stall_curve, ':', 'Color', orange, 'LineWidth', 1.4, 'DisplayName', 'stall cap, $q S_v \eta_v C_{L,max}$');
plot(V_D, Yv, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7, 'HandleVisibility', 'off');
xline(V_D, ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
yl = ylim; text(V_D, yl(1)+0.03*(yl(2)-yl(1)), sprintf('$V_D=%.1f$', V_D), ...
    'HorizontalAlignment', 'center', 'Color', [0.35 0.35 0.35], 'FontSize', 9, 'Interpreter', 'latex');
xlabel('equivalent airspeed V (m/s)'); ylabel('fin side force Y_v (N)');
title(sprintf('Fin side load: %.1f N at $V_D$ (%s)', Yv, local_ternary(capped_rudder,'stall-capped','rudder auth.')), 'Interpreter', 'latex');
legend('Location', 'best', 'Interpreter', 'latex');

nexttile; hold on; box on; grid on;
yyaxis left
plot(z, w_fin, '-', 'LineWidth', 1.6, 'DisplayName', 'running load w');
ylabel('running load w (N/m)');
yyaxis right
plot(z, M_fin, '-', 'LineWidth', 1.6, 'DisplayName', 'bending M');
ylabel('bending M (N$\cdot$m)', 'Interpreter', 'latex');
ylim([0, max(M_fin)*1.1]);
xlabel('height above fin root z (m)');
title(sprintf('Fin: root M = %.2f N*m (height %.0f mm), elliptical load', M_fin(1), H*1000));
legend('Location', 'best', 'Interpreter', 'latex');

nexttile; hold on; box on; grid on;
plot(x_tc, T_tc, '--', 'Color', teal, 'LineWidth', 1.6, 'DisplayName', 'torsion T');
plot(x_tc, M_l_tc, '-', 'Color', teal, 'LineWidth', 1.8, 'DisplayName', 'lateral bending M_l');
ylim([0, max(M_l_tc)*1.1]);
xlabel('station along the tailcone, from the fuselage root (m)'); ylabel('N$\cdot$m', 'Interpreter', 'latex');
title(sprintf('Tailcone: T = %.2f N*m (constant), $M_l$(root) = %.2f N*m', T_tailcone, M_l_tailcone), 'Interpreter', 'latex');
legend('Location', 'best', 'Interpreter', 'latex');

nexttile; axis off;
text(0, 0.9, 'Assumptions carried into this figure:', 'FontWeight', 'bold', 'FontSize', 10);
text(0, 0.7, sprintf('eta_h = eta_v = %.1f  (tail efficiency, not modelled elsewhere)', eta_h), 'FontSize', 9);
text(0, 0.55, sprintf('CL_{max,tail} = %.2f  (NACA 0010, ASSUMPTION)', CL_max_tail), 'FontSize', 9);
text(0, 0.4, sprintf('D_{fus,tail} = %.3f m  (tailboom diameter at the fin, ASSUMPTION)', D_fus), 'FontSize', 9);
text(0, 0.25, sprintf('l_{tc} ~ l_t = %.3f m  (tailcone arm approximated by the wing-to-tail arm)', l_tc), 'FontSize', 9);

sgtitle('Asymmetric case: full rudder, design point at $V_D$; ultimate 1.5$\times$', ...
    'Interpreter', 'latex', 'FontWeight', 'bold');

%% ---- Outputs ----
outputs.TailLoads.tail = struct( ...
    'horizontal_tail_governing_speed', V_h_governing, 'horizontal_tail_governing_label', point_labels{i_gov_h}, ...
    'horizontal_tail_capped', capped_points(i_gov_h), 'horizontal_tail_L_design', L_h_design_signed, ...
    'horizontal_tail_M_root', M_h(1), 'horizontal_tail_T_root', T_h(1), ...
    'fin_Yv', Yv, 'fin_capped', capped_rudder, 'fin_M_root', M_fin(1), ...
    'tailcone_T', T_tailcone, 'tailcone_Ml', M_l_tailcone, 'tailcone_zcp', z_cp);

% Carry the tail loads forward (D5 stress/sizing needs these):
params.structures.horizontal_tail_M_root = M_h(1);
params.structures.horizontal_tail_T_root = T_h(1);
params.structures.fin_M_root   = M_fin(1);
params.structures.tailcone_T   = T_tailcone;
params.structures.tailcone_Ml  = M_l_tailcone;

end

function out = local_ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

function [Q, R] = local_tipIntegrate(y, q)
    % Q(y) = int_y^s q(y') dy';  R(y) = int_y^s Q(y') dy'  (cantilever,
    % integrated from the free tip; BCs Q(s)=R(s)=0). Reused for
    % load->shear->moment and for running-torque->torque (take Q only).
    Total_q = trapz(y, q);
    Q = Total_q - cumtrapz(y, q);
    Total_Q = trapz(y, Q);
    R = Total_Q - cumtrapz(y, Q);
end
