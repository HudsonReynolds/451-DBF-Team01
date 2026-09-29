function [outputs, params] = WingLoads(params)

%% A8 Deliverable 3 -- Wings

%% Initializations
rho  = params.env.rho;
g    = params.env.g;
S    = params.geometry.S_wing;
c    = params.geometry.c_wing;

% Mass table is the source of truth for structural analysis (team
% decision): use its own sum as the aircraft weight here, not the
% iterated params.performance.MTOW, so wing/tail/fuselage all size
% against the same, consistent total. Once real masses replace the
% placeholders this comparison tells you if the two have drifted apart.
massTable = readtable('SizingParams.xlsx', 'Sheet', 'MassBudget');
MTOM = sum(massTable.Mass_g)/1000; % kg
MTOW = MTOM * g; % N

%% Wing Computations of Load, Shear, Bending Torsion at Positive, Negative, and Maneuver Load Cases
n_pos = params.performance.n_limit_pos;
n_neg = params.performance.n_limit_neg;
V_A   = params.performance.V_A;

V_S_neg = sqrt(2*MTOW/(rho*S*abs(params.aero.CL_max_neg)));
V_G = V_S_neg*sqrt(abs(n_neg));

b = params.geometry.wingspan;
s = b/2;

y = linspace(0, s, 300);
c_ell = (4*S/(pi*b)) * sqrt(max(1 - (2*y/b).^2, 0)); 
c_S = 0.5*(c + c_ell);                               
IcS = trapz(y, c_S);                                 

e_arm = (params.structures.x_spar - params.geometry.x_ac) * c; % ASSUMPTION on x_spar; sign decides whether Cm_ac adds to or opposes the lift-offset torque (discussion Q4)

% ---- Case 1: positive symmetric at V_A ----
k_pos = (n_pos*MTOW/2) / IcS; % STATE this normalising factor
w_pos = k_pos*c_S;
[V_pos, M_pos] = local_tipIntegrate(y, w_pos);

q_A = 0.5*rho*V_A^2;
t_pos = w_pos*e_arm + q_A*c^2*params.aero.CM_ac_w;
T_pos = local_tipIntegrate(y, t_pos);

% ---- Case 2: negative symmetric at V_G ----
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

t_roll_down = w_roll_down*e_arm + q_A*c^2*params.aero.CM_ac_w;
t_roll_up   = w_roll_up*e_arm   + q_A*c^2*params.aero.CM_ac_w;
T_roll_down = local_tipIntegrate(y, t_roll_down);
T_roll_up   = local_tipIntegrate(y, t_roll_up);

%% Console Prints and Plotting

% Driving (peak-magnitude) bending, shear and torsion across all four
% loading cases, WHEREVER along the span they actually occur -- not
% assumed to be the root, even though that is where they land out for
% this Shrenk-type loading (checked, not assumed: see the locations
% printed below).
case_labels = {'positive symmetric', 'negative symmetric', 'rolling maneuver (down-going wing)', 'rolling maneuver (up-going wing)'};
M_cases = {M_pos, M_neg, M_roll_down, M_roll_up};
V_cases = {V_pos, V_neg, V_roll_down, V_roll_up};
T_cases = {T_pos, T_neg, T_roll_down, T_roll_up};

[M_drive, y_M_drive, iM_case] = local_drivingAcrossCases(y, M_cases);
[V_drive, y_V_drive, iV_case] = local_drivingAcrossCases(y, V_cases);
[T_drive, y_T_drive, iT_case] = local_drivingAcrossCases(y, T_cases);

tip_residual = max(abs([M_pos(end) M_neg(end) M_roll_down(end) M_roll_up(end)]));
if tip_residual > 1e-6
    warning('WingLoads:tipBC', 'Free-tip boundary condition not satisfied: |residual| = %.2e N*m (should be ~0).', tip_residual);
end

fprintf('\n--- Wing Loads (Deliverable 3) ---\n');
fprintf('  Driving bending:  M = %+.2f N*m  at y = %.3f m  (%s)\n', M_drive, y_M_drive, case_labels{iM_case});
fprintf('  Driving shear:    V = %+.2f N    at y = %.3f m  (%s)\n', V_drive, y_V_drive, case_labels{iV_case});
fprintf('  Driving torsion:  T = %+.3f N*m  at y = %.3f m  (%s)\n', T_drive, y_T_drive, case_labels{iT_case});

blue = [0.00 0.45 0.74]; orange = [0.85 0.33 0.10]; red = [0.80 0.00 0.00]; % red = driving/critical load marker

% ---- Figure: Case 1, positive symmetric ----
[w_pos_pk, y_w_pos_pk] = local_driving(y, w_pos);
[V_pos_pk, y_V_pos_pk] = local_driving(y, V_pos);
[M_pos_pk, y_M_pos_pk] = local_driving(y, M_pos);
[T_pos_pk, y_T_pos_pk] = local_driving(y, T_pos);

figure('Name','Wing Loads - Positive Symmetric','Color','w','WindowStyle','docked');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile; hold on; box on; grid on;
plot(y, w_pos, '-', 'Color', blue, 'LineWidth', 1.6, 'DisplayName', 'Shrenk: $w=k\,c_S$');
plot(y, k_pos*c*ones(size(y)), 'k--', 'DisplayName', 'planform chord (rectangular, no taper)');
plot(y, k_pos*c_ell, 'r:', 'DisplayName', 'elliptical shape');
plot(y_w_pos_pk, w_pos_pk, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('running load w (N/m)');
title(sprintf('Running load at n=%+.2f', n_pos));
legend('Location', 'best', 'Interpreter', 'latex');

nexttile; hold on; box on; grid on;
plot(y, V_pos, 'Color', blue, 'LineWidth', 1.6);
plot(y_V_pos_pk, V_pos_pk, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('shear V (N)');
title(sprintf('Shear: driving V=%.1f N at y=%.3f m', V_pos_pk, y_V_pos_pk));

nexttile; hold on; box on; grid on;
plot(y, M_pos, 'Color', blue, 'LineWidth', 1.6);
plot(y_M_pos_pk, M_pos_pk, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('bending moment M (N$\cdot$m)', 'Interpreter', 'latex');
title(sprintf('Bending: driving M=%.2f N*m at y=%.3f m', M_pos_pk, y_M_pos_pk));

nexttile; hold on; box on; grid on;
plot(y, t_pos, '--', 'Color', blue, 'LineWidth', 1.4, 'DisplayName', 'running torque t (N$\cdot$m/m)');
plot(y, T_pos, '-', 'Color', blue, 'LineWidth', 1.8, 'DisplayName', 'torque T (N$\cdot$m)');
plot(y_T_pos_pk, T_pos_pk, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('N$\cdot$m/m  or  N$\cdot$m', 'Interpreter', 'latex');
title(sprintf('Torque: driving T=%.2f N*m at y=%.3f m', T_pos_pk, y_T_pos_pk));
legend('Location', 'best', 'Interpreter', 'latex');

sgtitle(sprintf('Wing Loads: Case 1 -- Positive Symmetric at $V_A$=%.1f m/s', V_A), 'Interpreter', 'latex');

% ---- Figure: Case 2, negative symmetric ----
[w_neg_pk, y_w_neg_pk] = local_driving(y, w_neg);
[V_neg_pk, y_V_neg_pk] = local_driving(y, V_neg);
[M_neg_pk, y_M_neg_pk] = local_driving(y, M_neg);
[T_neg_pk, y_T_neg_pk] = local_driving(y, T_neg);

figure('Name','Wing Loads - Negative Symmetric','Color','w','WindowStyle','docked');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile; hold on; box on; grid on;
plot(y, w_neg, '-', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', 'Shrenk: $w=k\,c_S$');
plot(y, k_neg*c*ones(size(y)), 'k--', 'DisplayName', 'planform chord (rectangular, no taper)');
plot(y, k_neg*c_ell, 'r:', 'DisplayName', 'elliptical shape');
plot(y_w_neg_pk, w_neg_pk, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('running load w (N/m)');
title(sprintf('Running load at n=%+.2f', n_neg));
legend('Location', 'best', 'Interpreter', 'latex');

nexttile; hold on; box on; grid on;
plot(y, V_neg, 'Color', orange, 'LineWidth', 1.6);
plot(y_V_neg_pk, V_neg_pk, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('shear V (N)');
title(sprintf('Shear: driving V=%.1f N at y=%.3f m', V_neg_pk, y_V_neg_pk));

nexttile; hold on; box on; grid on;
plot(y, M_neg, 'Color', orange, 'LineWidth', 1.6);
plot(y_M_neg_pk, M_neg_pk, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('bending moment M (N$\cdot$m)', 'Interpreter', 'latex');
title(sprintf('Bending: driving M=%.2f N*m at y=%.3f m', M_neg_pk, y_M_neg_pk));

nexttile; hold on; box on; grid on;
plot(y, t_neg, '--', 'Color', orange, 'LineWidth', 1.4, 'DisplayName', 'running torque t (N$\cdot$m/m)');
plot(y, T_neg, '-', 'Color', orange, 'LineWidth', 1.8, 'DisplayName', 'torque T (N$\cdot$m)');
plot(y_T_neg_pk, T_neg_pk, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
yline(0, 'k-', 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('N$\cdot$m/m  or  N$\cdot$m', 'Interpreter', 'latex');
title(sprintf('Torque: driving T=%.2f N*m at y=%.3f m', T_neg_pk, y_T_neg_pk));
legend('Location', 'best', 'Interpreter', 'latex');

sgtitle(sprintf('Wing Loads: Case 2 -- Negative Symmetric at $V_G$=%.1f m/s', V_G), 'Interpreter', 'latex');

% ---- Figure: Case 3, rolling maneuver ----
% Red marker sits on whichever of down-/up-going is more critical for
% that specific quantity (they can differ from each other and from Case
% 1/2's driving locations).
[w_roll_pk, y_w_roll_pk] = local_drivingAcrossCases(y, {w_roll_down, w_roll_up});
[V_roll_pk, y_V_roll_pk] = local_drivingAcrossCases(y, {V_roll_down, V_roll_up});
[M_roll_pk, y_M_roll_pk] = local_drivingAcrossCases(y, {M_roll_down, M_roll_up});
[T_roll_pk, y_T_roll_pk] = local_drivingAcrossCases(y, {T_roll_down, T_roll_up});

figure('Name','Wing Loads - Rolling Maneuver','Color','w','WindowStyle','docked');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile; hold on; box on; grid on;
plot(y, w_roll_down, '-', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', 'down-going wing');
plot(y, w_roll_up, '--', 'Color', orange, 'LineWidth', 1.4, 'DisplayName', 'up-going wing');
plot(y, w_pos, ':', 'Color', blue, 'LineWidth', 1.2, 'DisplayName', 'baseline (no aileron)');
plot(y_w_roll_pk, w_roll_pk, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('running load w (N/m)');
title('Running load');
legend('Location', 'best');

nexttile; hold on; box on; grid on;
plot(y, V_roll_down, '-', 'Color', orange, 'LineWidth', 1.6);
plot(y, V_roll_up, '--', 'Color', orange, 'LineWidth', 1.4);
plot(y_V_roll_pk, V_roll_pk, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('shear V (N)');
title(sprintf('Shear: driving V=%.1f N at y=%.3f m', V_roll_pk, y_V_roll_pk));

nexttile; hold on; box on; grid on;
plot(y, M_roll_down, '-', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', 'down-going wing');
plot(y, M_roll_up, '--', 'Color', orange, 'LineWidth', 1.4, 'DisplayName', 'up-going wing');
plot(y_M_roll_pk, M_roll_pk, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('bending moment M (N$\cdot$m)', 'Interpreter', 'latex');
title(sprintf('Bending: driving M=%.2f N*m at y=%.3f m', M_roll_pk, y_M_roll_pk));
legend('Location', 'best');

nexttile; hold on; box on; grid on;
plot(y, T_roll_down, '-', 'Color', orange, 'LineWidth', 1.6, 'DisplayName', 'down-going wing');
plot(y, T_roll_up, '--', 'Color', orange, 'LineWidth', 1.4, 'DisplayName', 'up-going wing');
plot(y_T_roll_pk, T_roll_pk, 'o', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
xlabel('spanwise station y (m)'); ylabel('torque T (N$\cdot$m)', 'Interpreter', 'latex');
title(sprintf('Torque: driving T=%.2f N*m at y=%.3f m', T_roll_pk, y_T_roll_pk));
legend('Location', 'best');

sgtitle(sprintf('Wing Loads: Case 3 -- Rolling Maneuver at $V_A$=%.1f m/s, full aileron (%.0f deg)', V_A, params.geometry.delta_a_limit), 'Interpreter', 'latex');
% Torque here omits the aileron's own deflection-induced pitching moment
% about the AC (a distinct Cm_delta effect) -- see the note printed above.

%% Outputted Values
outputs.WingLoads.wing = struct( ...
    'case_labels', {case_labels}, ...
    'M_driving', M_drive, 'M_driving_location', y_M_drive, 'M_driving_case', case_labels{iM_case}, ...
    'V_driving', V_drive, 'V_driving_location', y_V_drive, 'V_driving_case', case_labels{iV_case}, ...
    'T_driving', T_drive, 'T_driving_location', y_T_drive, 'T_driving_case', case_labels{iT_case}, ...
    'M_root_pos', M_pos(1), 'M_root_neg', M_neg(1), 'M_root_roll_down', M_roll_down(1), 'M_root_roll_up', M_roll_up(1), ...
    'T_root_pos', T_pos(1), 'T_root_neg', T_neg(1), 'T_root_roll_down', T_roll_down(1), 'T_root_roll_up', T_roll_up(1));

% Carry the driving loads forward (D5 stress/sizing must size against
% whichever case and station actually govern, not just the root of the
% positive case):
params.structures.wing_M_driving          = M_drive;
params.structures.wing_M_driving_location = y_M_drive;
params.structures.wing_governing_case     = case_labels{iM_case}; % bending governs spar sizing -- this is THE wing "governing case"
params.structures.wing_V_driving          = V_drive;
params.structures.wing_V_driving_location = y_V_drive;
params.structures.wing_T_driving          = T_drive; % excludes the unmodelled aileron Cm_delta contribution -- see notes above
params.structures.wing_T_driving_location = y_T_drive;

end

function [val, loc] = local_driving(y, q)
    % The signed value and station of q's largest-magnitude point.
    [~, idx] = max(abs(q));
    val = q(idx);
    loc = y(idx);
end

function [val, loc, iCase] = local_drivingAcrossCases(y, caseArrays)
    % Same as local_driving, but across several q(y) arrays sharing the
    % same y grid (e.g. one per loading case) -- returns which one won.
    val = NaN; loc = NaN; iCase = NaN; bestAbs = -Inf;
    for k = 1:numel(caseArrays)
        [v, l] = local_driving(y, caseArrays{k});
        if abs(v) > bestAbs
            bestAbs = abs(v); val = v; loc = l; iCase = k;
        end
    end
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
