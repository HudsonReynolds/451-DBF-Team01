function data = ControlSurfaceSizing(data)
% Deliverable 4 - Control surface sizing & authority (elevator, aileron, rudder)
%
% PLACEHOLDER ASSUMPTIONS to reconcile with the team's RFP/design choices:
%   - delta_e_limit, delta_a_limit, delta_r_limit : mechanical travel limits
%   - E_a, y1_frac, y2_frac                        : aileron chord/span fractions
%   - pb_2V_target, V_roll                         : roll rate target + design airspeed + SOURCE
%   - V_v, AR_v, E_r                               : vertical tail geometry, rudder chord fraction
%   - crosswind_ratio                              : design crosswind, V_xwind/V_TO

%% ---- Elevator: forward-CG landing-flare check ----
delta_e_limit = 15; % [deg] PLACEHOLDER -- confirm against servo/horn travel

CL_delta_e_Tail = data.CL_Alpha_Tail/pi * (acos(1-2*data.E) + 2*sqrt(data.E*(1-data.E)));
CL_delta_e      = data.St_S * CL_delta_e_Tail;
CM_delta_e      = CL_delta_e_Tail*data.St_S*(data.x_cg_design - data.x_ac) - CL_delta_e_Tail*data.V_H;
CM_Alpha        = -data.CL_Alpha*data.SM;

delta_e_flare = -1*((data.CM_ac_w*data.CL_Alpha + CM_Alpha*(data.CL_max - data.CL_0)) / ...
                     (data.CL_Alpha*CM_delta_e - CL_delta_e*CM_Alpha));
delta_e_flare_deg = rad2deg(delta_e_flare);
elevator_margin   = delta_e_limit - abs(delta_e_flare_deg);

fprintf('--- Elevator ---\n');
fprintf('Chord fraction E = %.2f, at CG = %.2f, CL_max = %.2f:\n', data.E, data.x_cg_design, data.CL_max);
fprintf('  delta_e required = %.2f deg (limit +-%.0f deg, margin = %.2f deg)\n', delta_e_flare_deg, delta_e_limit, elevator_margin);
if elevator_margin < 1
    warning('Elevator authority has under 1 deg of margin at the flare condition -- treat as a finding, not a pass.');
end

%% ---- Ailerons: roll-rate authority ----
E_a = 0.25;                     % aileron chord fraction -- PLACEHOLDER
y1_frac = 0.6; y2_frac = 0.95;  % aileron span fractions of the semispan -- PLACEHOLDER
delta_a_limit = deg2rad(20);    % PLACEHOLDER max aileron deflection
V_roll = data.V_C;              % PLACEHOLDER design airspeed for the roll criterion
pb_2V_target = 0.09;            % PLACEHOLDER roll-rate target -- state source (e.g. MIL-F-8785C Level 1)

b = data.S_wing / data.c_w;     % wingspan [m] -- rectangular wing (c_w constant along span)
y  = linspace(0, b/2, 400);
c_of_y = data.c_w*ones(size(y)); % rectangular wing: no taper, so no taper approximation enters this integral

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
V_v = 0.04;                   % vertical tail volume coefficient -- PLACEHOLDER, cite a source table
AR_v = 1.5;                   % vertical tail aspect ratio -- PLACEHOLDER
E_r = 0.30;                   % rudder chord fraction -- PLACEHOLDER
delta_r_limit = deg2rad(20);  % PLACEHOLDER max rudder deflection
crosswind_ratio = 0.2;        % V_crosswind/V_TO -- PLACEHOLDER, state design ratio

l_v = data.l_t; % assume the vertical tail shares the horizontal tail's moment arm -- PLACEHOLDER
S_v = V_v*data.S_wing*b/l_v;
h_v = sqrt(AR_v*S_v);

CL_Alpha_VT = CalcLiftSlope(AR_v, 6.29); % assumes same tail airfoil selection as the horizontal stabiliser

tau_r = (1/pi)*(acos(1-2*E_r) + 2*sqrt(E_r*(1-E_r)));
assert(tau_r >= 0 && tau_r <= 1, 'Rudder effectiveness out of bounds -- check E_r');

Cn_beta_VT = CL_Alpha_VT*V_v;         % vertical-tail-alone contribution (fuselage/wing terms belong in D5)
Cn_delta_r = -CL_Alpha_VT*V_v*tau_r;

beta_crosswind = asin(crosswind_ratio);
delta_r_required = -(Cn_beta_VT/Cn_delta_r)*beta_crosswind;
rudder_margin = rad2deg(delta_r_limit) - abs(rad2deg(delta_r_required));

fprintf('--- Rudder ---\n');
fprintf('S_v = %.4f m^2, AR_v = %.2f, chord fraction %.2f, crosswind ratio %.2f:\n', S_v, AR_v, E_r, crosswind_ratio);
fprintf('  delta_r required = %.2f deg (limit +-%.0f deg, margin = %.2f deg)\n', ...
    rad2deg(delta_r_required), rad2deg(delta_r_limit), rudder_margin);

data.ControlSurfaces.Elevator = struct('E', data.E, 'delta_flare_deg', delta_e_flare_deg, 'margin_deg', elevator_margin);
data.ControlSurfaces.Aileron  = struct('E_a', E_a, 'y1_frac', y1_frac, 'y2_frac', y2_frac, 'pb_2V', pb_2V_achieved, 'roll_rate_dps', rad2deg(p_roll));
data.ControlSurfaces.Rudder   = struct('S_v', S_v, 'AR_v', AR_v, 'E_r', E_r, 'delta_r_deg', rad2deg(delta_r_required), 'margin_deg', rudder_margin);

end
