% This script does initial sizing of the vehicle based on constraint
% analysis for the system.

% system parameters are all pulled from an Excel spreadsheet. See
% 'SizingParams.xlsx' for more details:

set(groot, 'defaultAxesTickLabelInterpreter','latex')

% load the values from the excel sheet
params = readcell("SizingParams.xlsx");

N = length(params);

% put the data into a MATLAB 
for idx = 2:N
    data.(params{idx,1}) = params{idx,2};
end

T_LF = (2*200.73)/data.V_C;
T_TU = (2*pi*50)/data.V_M;

rho_battery = 5.27E5; %Joule / kg
eta_m = 0.7; % revisit motor efficiency
P_m = eta_m * 1000; % revisit, might determined based off W/P; motor efficiency times 1kW battery

T_CL = sqrt(100^2+30^2) / (data.V_C); 
T_TO = 25 / data.V_TO * 2; % multiple by 2 for average speed during accelerating takeoff.
warmupN = 10;

%% Energy Consumption in Level Flight
LevelFlightBatteryWeightFraction = data.V_C * T_LF * data.g / (data.L_D_max) / data.eta_p / eta_m / rho_battery; %might want to change the L/DMax

%% Energy Consumption in Turning Flight
TurningBatteryWeightFraction = data.V_M * T_TU * data.n * data.g / (data.L_D_max) / data.eta_p / eta_m / rho_battery;%might want to change the L/DMax

%% Energy Consumption in Climbing Flight
%ClimbEnergyRequired = data.V_TO * W * T_CL * (cos(deg2rad(data.gamma)) / (data.L_D_max*.866) + sin(deg2rad(data.gamma)));

ClimbBatteryWeightFraction = data.V_TO * T_CL * data.g * (cos(deg2rad(data.gamma)) / (data.L_D_max*.866) + sin(deg2rad(data.gamma))) / data.eta_p / eta_m / rho_battery;%might want to change the L/DMax

%% Energy Consumption during Warmup and Takeoff
TakeoffEnergyRequired = P_m / eta_m * T_TO;

%% Battery Weight Fraction for Takeoff
TakeOffBatteryWeightFraction = T_TO * data.g / (eta_m * (data.W_P_design) * rho_battery); %gravity??

%% Battery Weight Fraction for Warmup
WarmUpBatteryWeightFraction = warmupN * TakeOffBatteryWeightFraction; %clarify /W or not

WeightFraction = .65; % Guess based on Chart in A3 assignment


BatteryWeightFraction = WarmUpBatteryWeightFraction + ...
    TakeOffBatteryWeightFraction + ClimbBatteryWeightFraction + ...
    TurningBatteryWeightFraction + LevelFlightBatteryWeightFraction

%WeightOfBattery = BatteryWeightFraction * VehicleWeight











