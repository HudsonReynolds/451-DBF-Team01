function CD_0 = DragBuildUp(data)

%% Fuselage
lambda = data.len_fuselage / data.width_fuselage; % Fineness ratio
FF_fuselage = 1 + 60/lambda^3 + lambda/400;

Re_fuselage = data.rho * data.V_C * data.len_fuselage / data.visc; % Fuselage Reynolds number, fuselage length as reference
Cf_fuselage = 0.455 / log10(Re_fuselage)^2.58;

CD_0_fuselage = Cf_fuselage * FF_fuselage * data.S_wet_fuselage / data.S_wing;

%% Wing
S_wet_wing = 2 * 1.02 * data.S_wing; % Wetted area [m^2]
FF_wing = 1 + (0.6/data.xcm_wing)*data.tc_wing + 100*data.tc_wing^4; % Form factor
Q_wing = 1; % Interference factor for high wing

Re_wing = data.rho * data.V_C * data.c_wing / data.visc; 
Cf_wing = 0.455 / log10(Re_wing)^2.58;

CD_0_wing = Cf_wing * FF_wing * Q_wing * S_wet_wing / data.S_wing; 

%% Horizontal stab
S_hstab = data.span_hstab * data.c_hstab; % Horizontal stab area [m^2]
S_wet_hstab = 2 * 1.02 * S_hstab; % Wetted area [m^2]
FF_hstab = 1 + (0.6/data.xcm_hstab)*data.tc_hstab + 100*data.tc_hstab^4; % Form factor
Q_hstab = 1.05; % Interference factor for tail

Re_hstab = data.rho * data.V_C * data.c_hstab / data.visc; 
Cf_hstab = 0.455 / log10(Re_hstab)^2.58;

CD_0_hstab = Cf_hstab * FF_hstab * Q_hstab * S_wet_hstab / data.S_wing;

%% Vertical stab
S_vstab = data.span_vstab * data.c_vstab; % Vertical stab area [m^2]
S_wet_vstab = 2 * 1.02 * S_vstab; % Wetted area [m^2]
FF_vstab = 1 + (0.6/data.xcm_vstab)*data.tc_vstab + 100*data.tc_vstab^4; % Form factor
Q_vstab = 1.05; % Interference factor for tail

Re_vstab = data.rho * data.V_C * data.c_vstab / data.visc;
Cf_vstab = 0.455 / log10(Re_vstab)^2.58;

CD_0_vstab = Cf_vstab * FF_vstab * Q_vstab * 2 * S_wet_vstab / data.S_wing; % 2x wetted area to account for H tail

%% Landing gear
% Main gear
A_main_wheel = data.main_wheel_dia * data.main_wheel_width; % [m^2]
A_main_strut = data.main_gear_len * data.main_strut_dia; % [m^2]

% Tail gear
A_tail_wheel = data.tail_wheel_dia * data.tail_wheel_width; % [m^2]
A_tail_strut = data.main_gear_len * data.main_strut_dia; % [m^2]

% Drag areas
fe_main_wheel = 2 * data.DqA_wheel * A_main_wheel;
fe_main_strut = 2 * data.DqA_strut * A_main_strut;
fe_tail_wheel = data.DqA_wheel * A_tail_wheel;
fe_tail_strut = data.DqA_strut * A_tail_strut;
fe = fe_main_wheel + fe_main_strut + fe_tail_wheel + fe_tail_strut;

CD_0_gear = fe / data.S_wing;

%% Parasitic drag build-up
CD_0_comp = CD_0_fuselage + CD_0_wing + CD_0_hstab + CD_0_vstab + CD_0_gear;
CD_0_misc = data.CD_0_leak_factor * CD_0_comp;
CD_0 = CD_0_comp + CD_0_misc

end

