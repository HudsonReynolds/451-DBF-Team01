function params = DragBuildUp(params)
%% Fuselage
lambda = params.geometry.len_fuselage / params.geometry.width_fuselage; % Fineness ratio
FF_fuselage = 1 + 60/lambda^3 + lambda/400;
Re_fuselage = params.env.rho * params.performance.V_C * params.geometry.len_fuselage / params.env.visc; % Fuselage Reynolds number, fuselage length as reference
Cf_fuselage = 0.455 / log10(Re_fuselage)^2.58;
CD_0_fuselage = Cf_fuselage * FF_fuselage * params.geometry.S_wet_fuselage / params.geometry.S_wing

%% Wing
S_wet_wing = 2 * params.aero.S_wet_factor * params.geometry.S_wing; % Wetted area [m^2]
FF_wing = 1 + (0.6/params.geometry.xcm_wing)*params.geometry.tc_wing + 100*params.geometry.tc_wing^4; % Form factor
Q_wing = params.aero.Q_wing; % Interference factor for high wing
Re_wing = params.env.rho * params.performance.V_C * params.geometry.c_wing / params.env.visc;
Cf_wing = 0.455 / log10(Re_wing)^2.58;
CD_0_wing = Cf_wing * FF_wing * Q_wing * S_wet_wing / params.geometry.S_wing

%% Horizontal stab
S_hstab = params.geometry.span_hstab * params.geometry.c_hstab; % Horizontal stab area [m^2]
S_wet_hstab = 2 * params.aero.S_wet_factor * S_hstab; % Wetted area [m^2]
FF_hstab = 1 + (0.6/params.geometry.xcm_hstab)*params.geometry.tc_hstab + 100*params.geometry.tc_hstab^4; % Form factor
Q_hstab = params.aero.Q_hstab; % Interference factor for tail
Re_hstab = params.env.rho * params.performance.V_C * params.geometry.c_hstab / params.env.visc;
Cf_hstab = 0.455 / log10(Re_hstab)^2.58;
CD_0_hstab = Cf_hstab * FF_hstab * Q_hstab * S_wet_hstab / params.geometry.S_wing

%% Vertical stab
S_vstab = params.geometry.span_vstab * params.geometry.c_vstab; % Vertical stab area [m^2]
S_wet_vstab = 2 * params.aero.S_wet_factor * S_vstab; % Wetted area [m^2]
FF_vstab = 1 + (0.6/params.geometry.xcm_vstab)*params.geometry.tc_vstab + 100*params.geometry.tc_vstab^4; % Form factor
Q_vstab = params.aero.Q_vstab; % Interference factor for tail
Re_vstab = params.env.rho * params.performance.V_C * params.geometry.c_vstab / params.env.visc;
Cf_vstab = 0.455 / log10(Re_vstab)^2.58;
CD_0_vstab = Cf_vstab * FF_vstab * Q_vstab * params.geometry.n_vfins * S_wet_vstab / params.geometry.S_wing % n_vfins x wetted area to account for H tail

%% Landing gear
% Main gear
A_main_wheel = params.geometry.main_wheel_dia * params.geometry.main_wheel_width; % [m^2]
A_main_strut = params.geometry.main_gear_len * params.geometry.main_strut_dia; % [m^2]

% Tail gear
A_tail_wheel = params.geometry.tail_wheel_dia * params.geometry.tail_wheel_width; % [m^2]
A_tail_strut = params.geometry.tail_gear_len * params.geometry.tail_strut_dia; % [m^2]

% Drag areas
fe_main_wheel = 2 * params.aero.DqA_wheel * A_main_wheel;
fe_main_strut = 2 * params.aero.DqA_strut * A_main_strut;
fe_tail_wheel = params.aero.DqA_wheel * A_tail_wheel;
fe_tail_strut = params.aero.DqA_strut * A_tail_strut;
fe = fe_main_wheel + fe_main_strut + fe_tail_wheel + fe_tail_strut;
CD_0_gear = fe / params.geometry.S_wing

%% Parasitic drag build-up
CD_0_comp = CD_0_fuselage + CD_0_wing + CD_0_hstab + CD_0_vstab + CD_0_gear;
CD_0_misc = params.aero.CD_0_misc_factor * CD_0_comp
CD_0 = CD_0_comp + CD_0_misc
params.aero.CD_0 = CD_0;

end