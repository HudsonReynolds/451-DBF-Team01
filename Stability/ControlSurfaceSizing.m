function [outputs, params] = ControlSurfaceSizing(params)

% ---- Elevator: trim envelope plot (matches the assignment's example figure) ----
delta_e_limit = params.geometry.delta_e_limit_deg;

CG_cases  = [params.geometry.x_cg_design-.1, params.geometry.x_cg_aft];
CG_labels = {'Forward CG limit','Aft CG limit'};

CL_delta_e_Tail = params.aero.CL_Alpha_Tail/pi * (acos(1-2*params.geometry.E) + 2*sqrt(params.geometry.E*(1-params.geometry.E)));
CL_delta_e      = params.geometry.St_S * CL_delta_e_Tail;

% Neutral point (same derivation as StabilityDerivatives.m) so static
% margin -- and therefore CM_alpha -- is recomputed per CG case instead
% of reusing a single fixed data.SM:
x_n = params.geometry.x_ac + (params.aero.CL_Alpha_Tail*(1-params.aero.de_da)*params.geometry.V_H) / ...
      (params.aero.CL_Alpha_Wing + params.geometry.St_S*params.aero.CL_Alpha_Tail*(1-params.aero.de_da));

CL_range = linspace(-0.3, params.aero.CL_max, 200);

figure('Name','Trim Envelope - Elevator')
hold on
plot_colors = lines(numel(CG_cases));
delta_e_at_max_all = zeros(size(CG_cases));

for k = 1:numel(CG_cases)
    x_cg_k       = CG_cases(k);
    CM_Alpha_k   = -params.aero.CL_Alpha*(x_n - x_cg_k);
    CM_delta_e_k = CL_delta_e_Tail*params.geometry.St_S*(x_cg_k - params.geometry.x_ac) - CL_delta_e_Tail*params.geometry.V_H;

    delta_e_k_deg = rad2deg(-1*((params.aero.CM_0*params.aero.CL_Alpha + CM_Alpha_k*(CL_range - params.aero.CL_0)) / ...
                                 (params.aero.CL_Alpha*CM_delta_e_k - CL_delta_e*CM_Alpha_k)));

    plot(CL_range, delta_e_k_deg, 'LineWidth', 1.5, 'Color', plot_colors(k,:), 'DisplayName', CG_labels{k})
    delta_e_at_max_all(k) = interp1(CL_range, delta_e_k_deg, params.aero.CL_max);
end

yline(delta_e_limit, ':k', sprintf('Elevator travel limit, +-%d deg', delta_e_limit), 'HandleVisibility','off')
yline(-delta_e_limit, ':k', 'HandleVisibility','off')
xline(params.aero.CL_max, '-.', 'C_{L,max}', 'Color',[0 0.6 0.3], 'LineWidth',1.2, 'HandleVisibility','off')

[worst_deg, worst_idx] = max(abs(delta_e_at_max_all));
elevator_margin = delta_e_limit - worst_deg;
plot(params.aero.CL_max, delta_e_at_max_all(worst_idx), 'ko', 'MarkerFaceColor','k', 'MarkerSize',6, 'HandleVisibility','off')
text(params.aero.CL_max - 0.05, delta_e_at_max_all(worst_idx), ...
    sprintf('%s at C_{L,max}:\n%.0f of %d deg used, %.0f deg left', CG_labels{worst_idx}, worst_deg, delta_e_limit, elevator_margin), ...
    'HorizontalAlignment','right','VerticalAlignment','top')

xlabel('Trim Lift Coefficient, C_L (-)')
ylabel('Elevator Deflection to Trim, \delta_e (deg)')
title('Trim Envelope -- Elevator Required against Lift Coefficient')
legend('Location','best')
ylim([-1.5*delta_e_limit, 1.5*delta_e_limit])
grid on

fprintf('--- Elevator ---\n');
fprintf('  Worst-case delta_e at CL_max = %.2f deg (limit +-%.0f deg, margin = %.2f deg)\n', worst_deg, delta_e_limit, elevator_margin);
if elevator_margin < 1
    warning('Elevator authority has under 1 deg of margin at the flare condition -- treat as a finding, not a pass.');
end


%% ---- Ailerons: roll-rate authority ----

delta_a_limit = deg2rad(params.geometry.delta_a_limit);    % max aileron deflection
V_roll = 1.3*params.aero.V_S;                              % design airspeed for the roll criterion

y  = linspace(0, params.geometry.wingspan/2, 400);
c_of_y = params.geometry.c_wing*ones(size(y));             % rectangular wing: no taper, so no taper approximation enters this integral

tau_a = (1/pi)*(acos(1-2*params.geometry.E_a) + 2*sqrt(params.geometry.E_a*(1-params.geometry.E_a))); % Glauert flap effectiveness (exact thin-airfoil result, bounded 0-1)
assert(tau_a >= 0 && tau_a <= 1, 'Aileron effectiveness out of bounds -- check E_a');
Cl_delta_local = params.aero.CL_Alpha_Wing*tau_a;

y1 = params.geometry.y1_frac*params.geometry.wingspan/2; y2 = params.geometry.y2_frac*params.geometry.wingspan/2;
mask = (y >= y1) & (y <= y2);
Cl_deltaA = (2*Cl_delta_local/(params.geometry.S_wing*params.geometry.wingspan)) * trapz(y(mask), c_of_y(mask).*y(mask));

% Roll damping (Sadraey ch. 12 form) -- VERIFY exact leading coefficient against your text
Cl_p = -4*(params.aero.CL_Alpha_Wing + params.aero.CD_0)/(params.geometry.S_wing*params.geometry.wingspan^2) * trapz(y, c_of_y.*y.^2);

% Convert the stated deg/s target into the equivalent pb/2V at V_roll -- this
% line needs b, Cl_deltaA, and Cl_p to already exist, so it goes here, not
% up near roll_rate_target_dps:
pb_2V_achieved = -(Cl_deltaA/Cl_p)*delta_a_limit;
p_roll_dps = rad2deg(pb_2V_achieved*2*V_roll/params.geometry.wingspan);
roll_margin_dps = p_roll_dps - params.aero.roll_rate_target_dps;

fprintf('--- Ailerons ---\n');
fprintf('Span %.0f%%-%.0f%% semispan, chord fraction %.2f:\n', params.geometry.y1_frac*100, params.geometry.y2_frac*100, params.geometry.E_a);
fprintf('  roll rate achieved = %.1f deg/s (target %.0f deg/s) at %.1f m/s, margin = %.1f deg/s\n', ...
    p_roll_dps, params.aero.roll_rate_target_dps, V_roll, roll_margin_dps);
if abs(roll_margin_dps) < 2
    warning('Roll rate sits right on the target -- treat as a finding, not a pass.');
end

%% ---- Rudder: crosswind authority ----
% Sarah Inputs:
delta_r_limit = deg2rad(params.geometry.delta_r_limit);  % Max rudder deflection

l_v = params.geometry.l_t; % assume the vertical tail shares the horizontal tail's moment arm -- PLACEHOLDER
S_v = params.geometry.V_v*params.geometry.S_wing*params.geometry.wingspan/l_v;

params.aero.CL_Alpha_VT = CalcLiftSlope(params.geometry.AR_v, 6.29); % assumes same tail airfoil selection as the horizontal stabiliser

tau_r = (1/pi)*(acos(1-2*params.geometry.E_r) + 2*sqrt(params.geometry.E_r*(1-params.geometry.E_r)));
assert(tau_r >= 0 && tau_r <= 1, 'Rudder effectiveness out of bounds -- check E_r');

params.aero.Cn_beta_VT = params.aero.CL_Alpha_VT*params.geometry.V_v;         % vertical-tail-alone contribution (fuselage/wing terms belong in D5)
Cn_delta_r = -params.aero.CL_Alpha_VT*params.geometry.V_v*tau_r;

beta_crosswind = asin(params.aero.crosswind_ratio);
delta_r_required = -(params.aero.Cn_beta_VT/Cn_delta_r)*beta_crosswind;
rudder_margin = rad2deg(delta_r_limit) - abs(rad2deg(delta_r_required));

fprintf('--- Rudder ---\n');
fprintf('S_v = %.4f m^2, AR_v = %.2f, chord fraction %.2f, crosswind ratio %.2f:\n', S_v, params.geometry.AR_v, params.geometry.E_r, params.aero.crosswind_ratio);
fprintf('  delta_r required = %.2f deg (limit +-%.0f deg, margin = %.2f deg)\n', ...
    rad2deg(delta_r_required), rad2deg(delta_r_limit), rudder_margin);

% Initialization Parameters
outputs.ControlSurfaces.Elevator = struct('E', params.geometry.E, 'delta_flare_deg', worst_deg, 'margin_deg', elevator_margin);
outputs.ControlSurfaces.Aileron  = struct('E_a', params.geometry.E_a, 'y1_frac', params.geometry.y1_frac, 'y2_frac', params.geometry.y2_frac, 'pb_2V', pb_2V_achieved, 'roll_rate_dps', p_roll_dps);
outputs.ControlSurfaces.Rudder   = struct('S_v', S_v, 'AR_v', params.geometry.AR_v, 'E_r', params.geometry.E_r, 'delta_r_deg', rad2deg(delta_r_required), 'margin_deg', rudder_margin);

end
