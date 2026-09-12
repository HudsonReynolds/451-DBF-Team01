function data = ControlSurfaceSizing(data)
% Deliverable 4 - Control surface sizing & authority (elevator, aileron, rudder)
%
% PLACEHOLDER ASSUMPTIONS to reconcile with the team's RFP/design choices:
%   - delta_e_limit, delta_a_limit, delta_r_limit : mechanical travel limits
%   - E_a, y1_frac, y2_frac                        : aileron chord/span fractions
%   - pb_2V_target, V_roll                         : roll rate target + design airspeed + SOURCE
%   - data.V_v, AR_v, E_r                               : vertical tail geometry, rudder chord fraction
%   - crosswind_ratio                              : design crosswind, V_xwind/V_TO

%% ---- Elevator: trim envelope plot (matches the assignment's example figure) ----
delta_e_limit = data.delta_e_limit_deg;

CG_cases  = [data.x_cg_design-.1, data.x_cg_aft];
CG_labels = {'Forward CG limit','Aft CG limit'};

CL_delta_e_Tail = data.CL_Alpha_Tail/pi * (acos(1-2*data.E) + 2*sqrt(data.E*(1-data.E)));
CL_delta_e      = data.St_S * CL_delta_e_Tail;

% Neutral point (same derivation as StabilityDerivatives.m) so static
% margin -- and therefore CM_alpha -- is recomputed per CG case instead
% of reusing a single fixed data.SM:
one_minus_deda = (data.CL_Alpha - data.CL_Alpha_Wing)/data.CL_Alpha_Tail;
x_n = data.x_ac + (data.CL_Alpha_Tail*one_minus_deda*data.V_H) / ...
      (data.CL_Alpha_Wing + data.St_S*data.CL_Alpha_Tail*one_minus_deda);

CL_range = linspace(-0.3, data.CL_max, 200);

figure('Name','Trim Envelope - Elevator')
hold on
plot_colors = lines(numel(CG_cases));
delta_e_at_max_all = zeros(size(CG_cases));

for k = 1:numel(CG_cases)
    x_cg_k       = CG_cases(k);
    CM_Alpha_k   = -data.CL_Alpha*(x_n - x_cg_k);
    CM_delta_e_k = CL_delta_e_Tail*data.St_S*(x_cg_k - data.x_ac) - CL_delta_e_Tail*data.V_H;

    delta_e_k_deg = rad2deg(-1*((data.CM_0*data.CL_Alpha + CM_Alpha_k*(CL_range - data.CL_0)) / ...
                                 (data.CL_Alpha*CM_delta_e_k - CL_delta_e*CM_Alpha_k)));

    plot(CL_range, delta_e_k_deg, 'LineWidth', 1.5, 'Color', plot_colors(k,:), 'DisplayName', CG_labels{k})
    delta_e_at_max_all(k) = interp1(CL_range, delta_e_k_deg, data.CL_max);
end

yline(delta_e_limit, ':k', sprintf('Elevator travel limit, +-%d deg', delta_e_limit), 'HandleVisibility','off')
yline(-delta_e_limit, ':k', 'HandleVisibility','off')
xline(data.CL_max, '-.', 'C_{L,max}', 'Color',[0 0.6 0.3], 'LineWidth',1.2, 'HandleVisibility','off')

[worst_deg, worst_idx] = max(abs(delta_e_at_max_all));
elevator_margin = delta_e_limit - worst_deg;
plot(data.CL_max, delta_e_at_max_all(worst_idx), 'ko', 'MarkerFaceColor','k', 'MarkerSize',6, 'HandleVisibility','off')
text(data.CL_max - 0.05, delta_e_at_max_all(worst_idx), ...
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

% Sarah Inputs
E_a = 0.25;                     % aileron chord fraction
y1_frac = 0.7; y2_frac = 1;     % aileron span fractions of the semispan


delta_a_limit = deg2rad(15);    % max aileron deflection
V_roll = 1.3*data.V_S;          % design airspeed for the roll criterion
% C_lp = -data.CL_Alpha_Wing/4;
% CL_Epsilon_A

pb_2V_target = 0.09;            % PLACEHOLDER roll-rate target -- state source (e.g. MIL-F-8785C Level 1)

b = data.S_wing / data.c_wing;     % wingspan [m] -- rectangular wing (c_wing constant along span)
y  = linspace(0, b/2, 400);
c_of_y = data.c_wing*ones(size(y)); % rectangular wing: no taper, so no taper approximation enters this integral

tau_a = (1/pi)*(acos(1-2*E_a) + 2*sqrt(E_a*(1-E_a))); % Glauert flap effectiveness (exact thin-airfoil result, bounded 0-1)
assert(tau_a >= 0 && tau_a <= 1, 'Aileron effectiveness out of bounds -- check E_a');
Cl_delta_local = data.CL_Alpha_Wing*tau_a;

y1 = y1_frac*b/2; y2 = y2_frac*b/2;
mask = (y >= y1) & (y <= y2);
Cl_deltaA = (2*Cl_delta_local/(data.S_wing*b)) * trapz(y(mask), c_of_y(mask).*y(mask));

% Roll damping (Sadraey ch. 12 form) -- VERIFY exact leading coefficient against your text
Cl_p = -4*(data.CL_Alpha_Wing + data.CD_0)/(data.S_wing*b^2) * trapz(y, c_of_y.*y.^2);

pb_2V_achieved = -(Cl_deltaA/Cl_p)*delta_a_limit;
p_roll = pb_2V_achieved*2*V_roll/b;
roll_margin = pb_2V_achieved - pb_2V_target;

fprintf('--- Ailerons ---\n');
fprintf('Span %.0f%%-%.0f%% semispan, chord fraction %.2f:\n', y1_frac*100, y2_frac*100, E_a);
fprintf('  pb/2V achieved = %.4f (target %.4f), roll rate = %.1f deg/s at %.1f m/s\n', ...
    pb_2V_achieved, pb_2V_target, rad2deg(p_roll), V_roll);
if abs(roll_margin) < 0.005
    warning('Roll rate sits right on the target -- treat as a finding, not a pass.');
end

%% ---- Rudder: crosswind authority ----
% Sarah Inputs:
Sv_Sw = data.V_v * data.wingspan / data.l_t;
Sv = Sv_Sw * data.S_wing;
E_r = 0.30;                   % rudder chord fraction -- PLACEHOLDER
AR_v = 1.5;                   % vertical tail aspect ratio -- PLACEHOLDER


delta_r_limit = deg2rad(20);  % PLACEHOLDER max rudder deflection
crosswind_ratio = 0.2;        % V_crosswind/V_TO -- PLACEHOLDER, state design ratio

l_v = data.l_t; % assume the vertical tail shares the horizontal tail's moment arm -- PLACEHOLDER
S_v = data.V_v*data.S_wing*b/l_v;
h_v = sqrt(AR_v*S_v);

CL_Alpha_VT = CalcLiftSlope(AR_v, 6.29); % assumes same tail airfoil selection as the horizontal stabiliser

tau_r = (1/pi)*(acos(1-2*E_r) + 2*sqrt(E_r*(1-E_r)));
assert(tau_r >= 0 && tau_r <= 1, 'Rudder effectiveness out of bounds -- check E_r');

Cn_beta_VT = CL_Alpha_VT*data.V_v;         % vertical-tail-alone contribution (fuselage/wing terms belong in D5)
Cn_delta_r = -CL_Alpha_VT*data.V_v*tau_r;

beta_crosswind = asin(crosswind_ratio);
delta_r_required = -(Cn_beta_VT/Cn_delta_r)*beta_crosswind;
rudder_margin = rad2deg(delta_r_limit) - abs(rad2deg(delta_r_required));

fprintf('--- Rudder ---\n');
fprintf('S_v = %.4f m^2, AR_v = %.2f, chord fraction %.2f, crosswind ratio %.2f:\n', S_v, AR_v, E_r, crosswind_ratio);
fprintf('  delta_r required = %.2f deg (limit +-%.0f deg, margin = %.2f deg)\n', ...
    rad2deg(delta_r_required), rad2deg(delta_r_limit), rudder_margin);

data.ControlSurfaces.Elevator = struct('E', data.E, 'delta_flare_deg', worst_deg, 'margin_deg', elevator_margin);
data.ControlSurfaces.Aileron  = struct('E_a', E_a, 'y1_frac', y1_frac, 'y2_frac', y2_frac, 'pb_2V', pb_2V_achieved, 'roll_rate_dps', rad2deg(p_roll));
data.ControlSurfaces.Rudder   = struct('S_v', S_v, 'AR_v', AR_v, 'E_r', E_r, 'delta_r_deg', rad2deg(delta_r_required), 'margin_deg', rudder_margin);

end
