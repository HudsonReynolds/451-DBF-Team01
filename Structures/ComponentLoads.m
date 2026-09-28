function [outputs, params] = ComponentLoads(params)

% A8 Deliverable 3 -- component loads: wing (lecture "08 - Structures",
% slides 15-21). Tail and fuselage loads to follow.
%
% Three candidate governing cases (slide 15), all computed so the
% governing one is picked from the numbers, not assumed:
%   1. positive symmetric at V_A  (CL_max and n+ meet at the corner)
%   2. negative symmetric at V_G  (the inverted mirror, same Shrenk shape
%      since it scales linearly with n; independently evaluated at its own
%      speed for the torque's dynamic-pressure term)
%   3. rolling maneuver at V_A, full aileron (slide 17: "usually governs
%      the wing ATTACHMENT") -- modelled as the baseline positive-symmetric
%      load plus/minus an aileron-induced lift increment over the aileron
%      span, using the SAME strip-theory model already used for roll-rate
%      sizing in ControlSurfaceSizing.m (Cl_delta_local = CL_Alpha_Wing*tau_a).
%      This is a reasonable preliminary model, not a full unsteady rolling
%      analysis -- state that when you report it.
%
% x_spar (wing spar/shear-centre chordwise location) is a new value this
% deliverable needs that nothing upstream produced -- it's flagged
% ASSUMPTION in Sheet1's "structures" subclass; replace it once the spar
% is chosen.

rho  = params.env.rho;
MTOW = params.performance.MTOW;
S    = params.geometry.S_wing;
c    = params.geometry.c_wing; % rectangular wing, constant chord

%% ================= WING =================
n_pos = params.performance.n_limit_pos;
n_neg = params.performance.n_limit_neg;
V_A   = params.performance.V_A;

% V_G (negative corner speed) isn't carried in params -- cheap to rederive,
% same formula as VnDiagram.m/VnOperatingEnvelope.m:
V_S_neg = sqrt(2*MTOW/(rho*S*abs(params.aero.CL_max_neg)));
V_G = V_S_neg*sqrt(abs(n_neg));

b = params.geometry.wingspan;
s = b/2; % semispan

y = linspace(0, s, 300);
c_ell = (4*S/(pi*b)) * sqrt(max(1 - (2*y/b).^2, 0)); % Shrenk's elliptical shape (slide 18)
c_S = 0.5*(c + c_ell);                                % Shrenk = mean of planform and ellipse
IcS = trapz(y, c_S);                                  % semispan integral (geometry only, same for every case)

e_arm = (params.structures.x_spar - params.geometry.x_ac) * c; % ASSUMPTION on x_spar; sign decides whether Cm_ac adds to or opposes the lift-offset torque (discussion Q4)

% ---- Case 1: positive symmetric at V_A ----
k_pos = (n_pos*MTOW/2) / IcS; % STATE this normalising factor
w_pos = k_pos*c_S;
[V_pos, M_pos] = local_tipIntegrate(y, w_pos);

q_A = 0.5*rho*V_A^2;
t_pos = w_pos*e_arm + q_A*c^2*params.aero.CM_ac_w;
T_pos = local_tipIntegrate(y, t_pos);

% ---- Case 2: negative symmetric at V_G ----
% (Shrenk shape is purely geometric -- it scales linearly with n, so no
% re-normalisation is needed; only the torque's dynamic-pressure term
% depends on the actual speed, which differs for this corner.)
k_neg = (n_neg*MTOW/2) / IcS;
w_neg = k_neg*c_S;
[V_neg, M_neg] = local_tipIntegrate(y, w_neg);

q_G = 0.5*rho*V_G^2;
t_neg = w_neg*e_arm + q_G*c^2*params.aero.CM_ac_w;
T_neg = local_tipIntegrate(y, t_neg);

% ---- Case 3: rolling maneuver, full aileron at V_A (slides 15, 17) ----
Cl_delta_local = params.aero.CL_Alpha_Wing * params.aero.tau_a; % same model as the roll-rate sizing
delta_a_max = deg2rad(params.geometry.delta_a_limit);
y1 = params.geometry.y1_frac*s; y2 = params.geometry.y2_frac*s;
aileron_mask = (y >= y1) & (y <= y2);

dw_aileron = zeros(size(y));
dw_aileron(aileron_mask) = q_A*c*Cl_delta_local*delta_a_max; % local lift increment over the aileron span

w_roll_down = w_pos + dw_aileron; % aileron trailing-edge down: more lift
w_roll_up   = w_pos - dw_aileron; % aileron trailing-edge up: less lift
[V_roll_down, M_roll_down] = local_tipIntegrate(y, w_roll_down);
[V_roll_up,   M_roll_up]   = local_tipIntegrate(y, w_roll_up);

% Torque: same lift-offset + section-pitching-moment formula as cases 1/2,
% applied to each panel's own load. This does NOT include the additional
% torque from the aileron's own deflection-induced pitching moment about
% the aerodynamic centre (a distinct "Cm_delta" effect, different from the
% Ch_alpha/Ch_delta HINGE-moment coefficients used for servo sizing in A7)
% -- that term isn't modelled here because no verified value/formula for it
% exists in this codebase yet. Get it from an XFLR5 flap-deflection sweep
% (or a handbook Cm_delta value) if you need the full torque.
t_roll_down = w_roll_down*e_arm + q_A*c^2*params.aero.CM_ac_w;
t_roll_up   = w_roll_up*e_arm   + q_A*c^2*params.aero.CM_ac_w;
T_roll_down = local_tipIntegrate(y, t_roll_down);
T_roll_up   = local_tipIntegrate(y, t_roll_up);

%% ---- Compare and identify the governing case ----
case_names = {'positive symmetric', 'negative symmetric', 'rolling maneuver (down-going wing)'};
M_roots = [M_pos(1), M_neg(1), M_roll_down(1)];
[M_governing, i_gov] = max(abs(M_roots));

fprintf('=== Wing loads: three candidate governing cases (slide 15) ===\n');
fprintf('  Shrenk shape semispan integral (geometry only) = %.4f m^2\n', IcS);
fprintf('  Case 1, positive symmetric at V_A=%.1f m/s (n=%+.2f): k=%.1f N/m^2, M(0)=%+.2f N*m, T(0)=%+.3f N*m\n', ...
    V_A, n_pos, k_pos, M_pos(1), T_pos(1));
fprintf('  Case 2, negative symmetric at V_G=%.1f m/s (n=%+.2f): k=%.1f N/m^2, M(0)=%+.2f N*m, T(0)=%+.3f N*m\n', ...
    V_G, n_neg, k_neg, M_neg(1), T_neg(1));
fprintf('  Case 3, rolling maneuver at V_A (full aileron, %.0f deg, over %.0f%%-%.0f%% semispan): down-going wing M(0)=%+.2f N*m, T(0)=%+.3f N*m; up-going wing M(0)=%+.2f N*m, T(0)=%+.3f N*m\n', ...
    params.geometry.delta_a_limit, params.geometry.y1_frac*100, params.geometry.y2_frac*100, M_roll_down(1), T_roll_down(1), M_roll_up(1), T_roll_up(1));
fprintf('    (strip-theory increment only, same model as the roll-rate sizing -- not a full unsteady rolling analysis)\n');
fprintf('    Case 3 torque includes the lift-offset + Cm_ac terms only -- NOT the aileron''s own deflection-induced\n');
fprintf('    pitching moment (a distinct Cm_delta effect, unmodelled here). Get it from an XFLR5 flap sweep for the full torque.\n');
fprintf('  BCs verified at the tip (all cases): max |residual| = %.2e N*m (should be ~0)\n', ...
    max(abs([M_pos(end) M_neg(end) M_roll_down(end) M_roll_up(end)])));
fprintf('  BCs at the root: deflection BCs v(0)=0, v''(0)=0 (not needed for loads)\n');
fprintf('  >>> GOVERNING CASE (largest |root bending|): %s, |M(0)| = %.2f N*m\n', case_names{i_gov}, M_governing);
if i_gov == 3
    fprintf('      Matches slide 17: the rolling maneuver drives the wing root -- and it drives the wing ATTACHMENT even harder,\n');
    fprintf('      since the attachment also reacts the left/right rolling moment, not just the lift.\n');
end

blue = [0.00 0.45 0.74]; orange = [0.85 0.33 0.10];

% ---- Figure: Case 1, positive symmetric ----
figure('Name','Wing Loads - Positive Symmetric','Color','w','Position',[100 100 1000 700],'WindowStyle','docked');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile; hold on; box on; grid on;
plot(y, w_pos, '-', 'Color', blue, 'LineWidth', 1.6, 'DisplayName', 'Shrenk: $w=k\,c_S$');
plot(y, k_pos*c*ones(size(y)), 'k--', 'DisplayName', 'planform chord (rectangular, no taper)');
plot(y, k_pos*c_ell, 'r:', 'DisplayName', 'elliptical shape');
xlabel('spanwise station y (m)'); ylabel('running load w (N/m)');
title(sprintf('Running load at n=%+.2f', n_pos));
legend('Location', 'best', 'Interpreter', 'latex');

nexttile; hold on; box on; grid on;
plot(y, V_pos, 'Color', blue, 'LineWidth', 1.6);
xlabel('spanwise station y (m)'); ylabel('shear V (N)');
title(sprintf('Shear, integrated from the tip: V(0)=%.1f N', V_pos(1)));

nexttile; hold on; box on; grid on;
plot(y, M_pos, 'Color', blue, 'LineWidth', 1.6);
xlabel('spanwise station y (m)'); ylabel('bending moment M (N$\cdot$m)', 'Interpreter', 'latex');
title(sprintf('Bending, integrated from the tip: M(0)=%.2f N*m', M_pos(1)));

nexttile; hold on; box on; grid on;
plot(y, t_pos, '--', 'Color', blue, 'LineWidth', 1.4, 'DisplayName', 'running torque t (N$\cdot$m/m)');
plot(y, T_pos, '-', 'Color', blue, 'LineWidth', 1.8, 'DisplayName', 'torque T (N$\cdot$m)');
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('N$\cdot$m/m  or  N$\cdot$m', 'Interpreter', 'latex');
title(sprintf('Torque, integrated from the tip: T(0)=%.2f N*m', T_pos(1)));
legend('Location', 'best', 'Interpreter', 'latex');

sgtitle(sprintf('Wing Loads: Case 1 -- Positive Symmetric at $V_A$=%.1f m/s', V_A), 'Interpreter', 'latex');

% ---- Figure: Case 2, negative symmetric ----
figure('Name','Wing Loads - Negative Symmetric','Color','w','Position',[100 100 1000 700],'WindowStyle','docked');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile; hold on; box on; grid on;
plot(y, w_neg, '-', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', 'Shrenk: $w=k\,c_S$');
plot(y, k_neg*c*ones(size(y)), 'k--', 'DisplayName', 'planform chord (rectangular, no taper)');
plot(y, k_neg*c_ell, 'r:', 'DisplayName', 'elliptical shape');
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('running load w (N/m)');
title(sprintf('Running load at n=%+.2f', n_neg));
legend('Location', 'best', 'Interpreter', 'latex');

nexttile; hold on; box on; grid on;
plot(y, V_neg, 'Color', orange, 'LineWidth', 1.6);
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('shear V (N)');
title(sprintf('Shear, integrated from the tip: V(0)=%.1f N', V_neg(1)));

nexttile; hold on; box on; grid on;
plot(y, M_neg, 'Color', orange, 'LineWidth', 1.6);
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('bending moment M (N$\cdot$m)', 'Interpreter', 'latex');
title(sprintf('Bending, integrated from the tip: M(0)=%.2f N*m', M_neg(1)));

nexttile; hold on; box on; grid on;
plot(y, t_neg, '--', 'Color', orange, 'LineWidth', 1.4, 'DisplayName', 'running torque t (N$\cdot$m/m)');
plot(y, T_neg, '-', 'Color', orange, 'LineWidth', 1.8, 'DisplayName', 'torque T (N$\cdot$m)');
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('N$\cdot$m/m  or  N$\cdot$m', 'Interpreter', 'latex');
title(sprintf('Torque, integrated from the tip: T(0)=%.2f N*m', T_neg(1)));
legend('Location', 'best', 'Interpreter', 'latex');

sgtitle(sprintf('Wing Loads: Case 2 -- Negative Symmetric at $V_G$=%.1f m/s', V_G), 'Interpreter', 'latex');

% ---- Figure: Case 3, rolling maneuver ----
figure('Name','Wing Loads - Rolling Maneuver','Color','w','Position',[100 100 1000 700],'WindowStyle','docked');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile; hold on; box on; grid on;
plot(y, w_roll_down, '-', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', 'down-going wing');
plot(y, w_roll_up, '--', 'Color', orange, 'LineWidth', 1.4, 'DisplayName', 'up-going wing');
plot(y, w_pos, ':', 'Color', blue, 'LineWidth', 1.2, 'DisplayName', 'baseline (no aileron)');
xlabel('spanwise station y (m)'); ylabel('running load w (N/m)');
title('Running load');
legend('Location', 'best');

nexttile; hold on; box on; grid on;
plot(y, V_roll_down, '-', 'Color', orange, 'LineWidth', 1.6);
plot(y, V_roll_up, '--', 'Color', orange, 'LineWidth', 1.4);
xlabel('spanwise station y (m)'); ylabel('shear V (N)');
title(sprintf('Shear: down %.1f N, up %.1f N at root', V_roll_down(1), V_roll_up(1)));

nexttile; hold on; box on; grid on;
plot(y, M_roll_down, '-', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', 'down-going wing');
plot(y, M_roll_up, '--', 'Color', orange, 'LineWidth', 1.4, 'DisplayName', 'up-going wing');
xlabel('spanwise station y (m)'); ylabel('bending moment M (N$\cdot$m)', 'Interpreter', 'latex');
title(sprintf('Bending: down %.1f, up %.1f N*m at root', M_roll_down(1), M_roll_up(1)));
legend('Location', 'best');

nexttile; hold on; box on; grid on;
plot(y, T_roll_down, '-', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', 'down-going wing');
plot(y, T_roll_up, '--', 'Color', orange, 'LineWidth', 1.4, 'DisplayName', 'up-going wing');
xlabel('spanwise station y (m)'); ylabel('torque T (N$\cdot$m)', 'Interpreter', 'latex');
title(sprintf('Torque: down %.2f, up %.2f N*m at root (lift-offset + $C_{m,ac}$ only)', T_roll_down(1), T_roll_up(1)), 'Interpreter', 'latex');
legend('Location', 'best');

sgtitle(sprintf('Wing Loads: Case 3 -- Rolling Maneuver at $V_A$=%.1f m/s, full aileron (%.0f deg)', V_A, params.geometry.delta_a_limit), 'Interpreter', 'latex');
% Torque here omits the aileron's own deflection-induced pitching moment
% about the AC (a distinct Cm_delta effect) -- see the note printed above.

%% ---- Outputs ----
outputs.ComponentLoads.wing = struct( ...
    'case_names', {case_names}, 'M_roots', M_roots, 'governing_case', case_names{i_gov}, 'M_root_governing', M_governing, ...
    'M_root_pos', M_pos(1), 'M_root_neg', M_neg(1), 'M_root_roll_down', M_roll_down(1), 'M_root_roll_up', M_roll_up(1), ...
    'T_root_pos', T_pos(1), 'T_root_neg', T_neg(1), 'T_root_roll_down', T_roll_down(1), 'T_root_roll_up', T_roll_up(1));

% Carry the GOVERNING case's loads forward (D5 stress/sizing must size
% against whichever case actually governs, not just the positive one):
T_candidates = [T_pos(1), T_neg(1), T_roll_down(1), T_roll_up(1)];
[~, i_T_gov] = max(abs(T_candidates));
params.structures.wing_M_root_governing = M_governing;
params.structures.wing_governing_case   = case_names{i_gov};
params.structures.wing_T_root_governing = T_candidates(i_T_gov); % excludes the unmodelled aileron Cm_delta contribution -- see notes above

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
