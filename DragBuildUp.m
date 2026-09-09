% This script estimates the parasitic drag build-up of the vehicle
close all;
clear;
clc;

set(groot, 'defaultAxesTickLabelInterpreter','latex');

% load the values from the excel sheet
params = readcell("SizingParams.xlsx");

N = length(params);

% put the data into a MATLAB 
for idx = 2:N
    data.(params{idx,1}) = params{idx,2};
end

%% Fuselage
len_fuselage = 1; % [m]
width_fuselage = 0.055; % [m]
lambda = len_fuselage / width_fuselage; % Fineness ratio

S_wet_fuselage = 0.127; % Fuselage wetted area, from config CAD [m^2]
FF_fuselage = 1 + 60/lambda^3 + lambda/400;

Re_fuselage = data.rho * data.V_C * len_fuselage / data.visc; % Fuselage Reynolds number, fuselage length as reference
Cf_fuselage = 0.455 / log10(Re_fuselage)^2.58;

CD_0_fuselage = Cf_fuselage * FF_fuselage * S_wet_fuselage / data.S_wing;

%% Wing
wingspan = 1.5; % [m]
wingspan_eff = wingspan - width_fuselage; % [m]
chord_wing = data.S_wing / wingspan_eff; % Wing chord [m]
t_c_wing = 0.15; % Airfoil thickness ratio, wing
x_c_m_wing = 0.3; % Station of max airfoil thickness, wing

S_wet_wing = 2 * 1.02 * data.S_wing; % Wetted area [m^2]
FF_wing = 1 + (0.6/x_c_m_wing)*t_c_wing + 100*t_c_wing^4; % Form factor
Q_wing = 1; % Interference factor for high wing

Re_wing = data.rho * data.V_C * chord_wing / data.visc; 
Cf_wing = 0.455 / log10(Re_wing)^2.58;

CD_0_wing = Cf_wing * FF_wing * Q_wing * S_wet_wing / data.S_wing; 

%% Horizontal stab
S_horz = 0.02; % Horizontal stab area [m^2]
chord_horz = chord_wing; % Horizontal stab chord [m]
t_c_horz = 0.12; % Airfoil thickness ratio, horizontal stab
x_c_m_horz = 0.3; % Station of max airfoil thickness, horizontal stab

S_wet_horz = 2 * 1.02 * S_horz; % Wetted area [m^2]
FF_horz = 1 + (0.6/x_c_m_horz)*t_c_horz + 100*t_c_horz^4; % Form factor
Q_horz = 1.05; % Interference factor for tail

Re_horz = data.rho * data.V_C * chord_horz / data.visc; 
Cf_horz = 0.455 / log10(Re_horz)^2.58;

CD_0_horz = Cf_horz * FF_horz * Q_horz * S_wet_horz / data.S_wing;

%% Vertical stab
S_vert = 0.02; % Vertical stab area [m^2]
chord_vert = chord_wing; % Vertical stab chord [m]
t_c_vert = 0.12; % Airfoil thickness ratio, vertical stab
x_c_m_vert = 0.3; % Station of max airfoil thickness, vertical stab

S_wet_vert = 2 * 1.02 * S_vert; % Wetted area [m^2]
FF_vert = 1 + (0.6/x_c_m_vert)*t_c_vert + 100*t_c_vert^4; % Form factor
Q_vert = 1.05; % Interference factor for tail

Re_vert = data.rho * data.V_C * chord_vert / data.visc;
Cf_vert = 0.455 / log10(Re_vert)^2.58;

CD_0_vert = Cf_wing * FF_wing * Q_vert * 2 * S_wet_vert / data.S_wing; % 2x wetted area to account for H tail

%% Landing gear
% Main gear dimensions
main_wheel_dia = 0.05; % [m]
main_wheel_width = 0.015; % [m]
main_gear_len = 0.155; % [m]
main_strut_dia = 0.016; % [m]
A_main_wheel = main_wheel_dia * main_wheel_width; % [m^2]
A_main_strut = main_gear_len * main_strut_dia; % [m^2]

% Tail gear dimensions
tail_wheel_dia = 0.05; % [m]
tail_wheel_width = 0.015; % [m]
tail_gear_len = 0.09; % [m]
tail_strut_dia = 0.01; % [m]
A_tail_wheel = tail_wheel_dia * tail_wheel_width; % [m^2]
A_tail_strut = main_gear_len * main_strut_dia; % [m^2]

% Drag factors
DqA_wheel = 0.25;
DqA_strut = 0.3;

% Drag areas
fe_main_wheel = 2 * DqA_wheel * A_main_wheel;
fe_main_strut = 2 * DqA_strut * A_main_strut;
fe_tail_wheel = DqA_wheel * A_tail_wheel;
fe_tail_strut = DqA_strut * A_tail_strut;
fe = fe_main_wheel + fe_main_strut + fe_tail_wheel + fe_tail_strut;

CD_0_gear = fe / data.S_wing;

%% Parasitic drag build-up
CD_0_comp = CD_0_fuselage + CD_0_wing + CD_0_horz + CD_0_vert + CD_0_gear;
CD_0_misc = 1.1 * CD_0_comp;
CD_0 = CD_0_comp + CD_0_misc

%% Oswald span efficiency
AR = wingspan / chord_wing;
e = 1.78 * (1 - 0.045*AR^0.68) - 0.64;
K = 1 / (pi * e * AR);

CL_max = 1.5;
CL = linspace(0, CL_max);

CD = CD_0 + K.*CL.^2;

plot(CD, CL);
grid on;
xlabel("C_D");
ylabel("C_L");
title("Clean Drag Polar");
