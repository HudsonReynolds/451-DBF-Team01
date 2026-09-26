function [outputs, params] = ServoSizing(params)

% A7 Deliverable 4 -- Servo sizing.
%
% Required torque: thin-airfoil (Glauert) hinge-moment theory, the same
% derivation as the "Hinge Moments" tab in SizingParams.xlsx, evaluated at
% the trim angle of attack for each airspeed (Ch depends on alpha as well
% as on delta_max). Inviscid/2-D theory typically over-predicts the real
% hinge moment by ~2x -- conservative for servo sizing.
%
% Available torque: two-point datasheet interpolation to the actual servo
% rail voltage, not the datasheet's highest quoted voltage.
%
% Torque at the servo = hinge moment / linkage ratio / linkage efficiency,
% times a stated safety-margin design factor (matches the linked Sheet1
% parameters added for the Hinge Moments tab).

NM2OZIN = 141.612; % N*m -> oz-in
servo_margin = 1.0; % extra safety-margin factor (design choice -- not linked to a Sheet1 parameter, same as the Hinge Moments tab)

%% ---- Per-surface geometry, pulled from params (Sheet1-linked where available) ----
cs(1) = struct( ...
    'name',          'Aileron', ...
    'c',             params.geometry.c_wing, ...
    'Ef',            params.geometry.E_a, ...
    'bf',            (params.geometry.y2_frac - params.geometry.y1_frac)*params.geometry.wingspan/2, ...
    'delta_max_deg', params.geometry.delta_a_limit, ...
    'ratio',         params.controls.linkage_ratio_aileron, ...
    'servo_V',       [4.8 6.0], ...   % PLACEHOLDER -- replace with the selected servo's datasheet voltage points
    'servo_T',       [40  52]);       % PLACEHOLDER -- oz-in stall torque at servo_V

cs(2) = struct( ...
    'name',          'Elevator', ...
    'c',             params.geometry.c_hstab, ...
    'Ef',            params.geometry.E, ...
    'bf',            params.geometry.span_hstab, ...
    'delta_max_deg', params.geometry.delta_e_limit_deg, ...
    'ratio',         params.controls.linkage_ratio_elevator, ...
    'servo_V',       [4.8 6.0], ...   % PLACEHOLDER -- elevator sees the largest hinge moment; a heavier-duty servo is assumed here
    'servo_T',       [70  90]);       % PLACEHOLDER -- oz-in stall torque at servo_V

cs(3) = struct( ...
    'name',          'Rudder', ...
    'c',             params.geometry.c_vstab, ...
    'Ef',            params.geometry.E_r, ...
    'bf',            params.geometry.span_vstab, ...   % per fin; one servo per fin assumed (n_vfins = 1)
    'delta_max_deg', params.geometry.delta_r_limit, ...
    'ratio',         params.controls.linkage_ratio_rudder, ...
    'servo_V',       [4.8 6.0], ...   % PLACEHOLDER -- replace with the selected servo's datasheet voltage points
    'servo_T',       [40  52]);       % PLACEHOLDER -- oz-in stall torque at servo_V

eta_linkage = params.controls.linkage_efficiency;
V_rail      = params.controls.V_servo_rail;

%% ---- Airspeed sweep ----
% TODO: swap the upper bound for V_NE / V_dive once A8 sets them; for now
% this runs from stall to twice the design cruise speed.
V = linspace(params.performance.V_S, 2*params.performance.V_C, 300);
q = 0.5*params.env.rho*V.^2;

% Trim CL at each airspeed (clipped at CL_max so the sweep doesn't run
% past stall), and the resulting local angle of attack:
CL_trim    = min(params.performance.MTOW ./ (q*params.geometry.S_wing), params.aero.CL_max);
alpha_wing = (CL_trim - params.aero.CL_0) / params.aero.CL_Alpha_Wing;   % rad, wing/aileron local AoA
alpha_tail = alpha_wing * (1 - params.aero.de_da);                      % rad, downwash-reduced tail/elevator AoA

alpha_of.Aileron  = alpha_wing;
alpha_of.Elevator = alpha_tail;
alpha_of.Rudder   = zeros(size(V)); % rudder Ch is driven by sideslip/deflection, not aircraft alpha, so it stays airspeed-independent here

%% ---- Required vs. available torque, per surface ----
figure('Color','w','Position',[100 100 1200 340]);
tiledlayout(1,3,'TileSpacing','compact','Padding','compact');

fprintf('=== Servo sizing: required vs. available torque (rail voltage = %.1f V) ===\n', V_rail);

for k = 1:numel(cs)
    s = cs(k);
    cf = s.Ef * s.c;
    Sf = cf * s.bf;

    % Thin-airfoil (Glauert) flap derivatives:
    theta_f  = acos(2*s.Ef - 1);
    X        = pi - theta_f;
    Ch_alpha = -(X*(cos(theta_f)-0.5) + sin(theta_f)*(1-cos(theta_f)/2)) / s.Ef^2;
    Ch_delta = -(X^2*(cos(theta_f)-0.5) + X*sin(theta_f) + sin(theta_f)^2/2) / (pi*s.Ef^2);

    alpha     = alpha_of.(s.name);
    delta_max = deg2rad(s.delta_max_deg);

    Ch = Ch_alpha*alpha + Ch_delta*delta_max;
    H  = Ch .* q .* Sf .* cf; % hinge moment [N*m]

    T_required = abs(H) * servo_margin * (s.ratio/eta_linkage) * NM2OZIN; % oz-in at the servo
    T_available = interp1(s.servo_V, s.servo_T, V_rail, 'linear', 'extrap') * ones(size(V));

    % Airspeed at which required first exceeds available:
    diff = T_required - T_available;
    if all(diff < 0)
        V_limit = NaN;
    elseif all(diff > 0)
        V_limit = V(1);
    else
        V_limit = interp1(diff, V, 0);
    end

    nexttile; hold on; box on; grid on;
    plot(V, T_required, 'b-', 'DisplayName', 'Required');
    yline(T_available(1), 'r--', 'DisplayName', 'Available');
    if ~isnan(V_limit)
        plot(V_limit, T_available(1), 'ko', 'MarkerFaceColor', 'k', 'HandleVisibility', 'off');
        text(V_limit, T_available(1), sprintf('  %.1f m/s', V_limit), 'VerticalAlignment', 'bottom');
    end
    xlabel('Airspeed [m/s]'); ylabel('Servo torque [oz-in]');
    title(s.name);
    legend('Location', 'best');

    if isnan(V_limit)
        fprintf('  %-9s required torque stays below %.1f oz-in available through %.1f m/s -- authority maintained.\n', ...
            [s.name ':'], T_available(1), V(end));
    elseif V_limit <= V(1)
        fprintf('  %-9s required torque already exceeds %.1f oz-in available at the sweep floor (%.1f m/s) -- servo undersized, reselect.\n', ...
            [s.name ':'], T_available(1), V(1));
    else
        fprintf('  %-9s runs out of authority above %.1f m/s (available = %.1f oz-in).\n', ...
            [s.name ':'], V_limit, T_available(1));
    end

    outputs.(s.name).V           = V;
    outputs.(s.name).T_required  = T_required;
    outputs.(s.name).T_available = T_available(1);
    outputs.(s.name).V_limit     = V_limit;
end

sgtitle('Required vs. Available Servo Torque');

end
