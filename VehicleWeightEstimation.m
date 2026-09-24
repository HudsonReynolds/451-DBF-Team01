function params = VehicleWeightEstimation(params)
T_LF = (2*params.performance.lap_length)/params.performance.V_C;
T_TU = (2*pi*params.performance.R_req)/params.performance.V_M;

% TODO: All of these in the excel
P_m = params.prop.eta_m * 1000; % revisit, might determined based off W/P; motor efficiency times 1kW battery
T_CL = sqrt(params.performance.climb_dist^2+params.performance.climb_alt^2) / (params.performance.V_TO);
T_TO = params.performance.S_TO / params.performance.V_TO * 2; % multiple by 2 for average speed during accelerating takeoff.

%% Energy Consumption in Level Flight
LevelFlightBatteryWeightFraction = params.performance.V_C * T_LF * params.env.g / params.aero.L_D_C / params.prop.eta_p_C / params.prop.eta_m / params.prop.rho_battery; %might want to change the L/DMax

%% Energy Consumption in Turning Flight
TurningBatteryWeightFraction = params.performance.V_M * T_TU * params.performance.n * params.env.g / params.aero.L_D_C / params.prop.eta_p_C / params.prop.eta_m / params.prop.rho_battery;%might want to change the L/DMax

%% Energy Consumption in Climbing Flight
%ClimbEnergyRequired = data.V_TO * W * T_CL * (cos(deg2rad(data.gamma)) / (data.L_D_max*.866) + sin(deg2rad(data.gamma)));
ClimbBatteryWeightFraction = params.performance.V_TO * T_CL * params.env.g * (cos(deg2rad(params.performance.gamma)) / (params.aero.L_D_C*.866) + sin(deg2rad(params.performance.gamma))) / params.prop.eta_p_C / params.prop.eta_m / params.prop.rho_battery;%might want to change the L/DMax

%% Energy Consumption during Warmup and Takeoff
TakeoffEnergyRequired = P_m / params.prop.eta_m * T_TO;

%% Battery Weight Fraction for Takeoff
TakeOffBatteryWeightFraction = T_TO * params.env.g / (params.prop.eta_m * params.prop.eta_p_TO * (params.performance.W_P_design) * params.prop.rho_battery); %gravity??

%% Battery Weight Fraction for Warmup
WarmUpBatteryWeightFraction = params.performance.warmupN * TakeOffBatteryWeightFraction; %clarify W or not
BatteryWeightFractionPlane = WarmUpBatteryWeightFraction + ...
    TakeOffBatteryWeightFraction + ClimbBatteryWeightFraction + ...
    TurningBatteryWeightFraction + LevelFlightBatteryWeightFraction;
% battery 20% margin:
BatteryWeightFraction = 1/params.prop.useableCapacity * BatteryWeightFractionPlane;
% temperature derate (table based on temperature, have operating temp as an
% input?)
BatteryWeightFraction = 1/params.prop.temp_derate * BatteryWeightFraction;

% plot over a range of weight values to find the solution:
W = linspace(1,7,100); % range of values [kg]
W_batt_payload = (BatteryWeightFraction)*W + params.performance.W_pay;
W_We = W - params.performance.W_e_frac*W;

% find the intersection of these lines for the empty weight:
[~,idx] = min(abs(W_batt_payload-W_We));
Weight = W(idx);
Weight_y = W_We(idx);
figure('Name','Total Vehicle Weight Estimate');
plot(W,W_batt_payload,'g', 'DisplayName', '$W_B + W_P$')
hold on
plot(W,W_We,'b', 'DisplayName', '$W - W_e$')
plot(Weight,Weight_y,'ro','DisplayName',string(Weight))
xlabel('Total Weight [kg]')
ylabel('Fractional Weights [kg]')
title('Weight Estimate')
legend('Location','Best');

% use total weight for energies
totBatteryWeight = BatteryWeightFraction * Weight
totBatteryEnergy = totBatteryWeight * params.prop.rho_battery
totEnergyRequiredByPlane = BatteryWeightFractionPlane * Weight * params.prop.rho_battery
totEnergyRequiredByBatt = totEnergyRequiredByPlane / (params.prop.eta_p_C*params.prop.eta_m)
energyLost = (1-params.prop.useableCapacity*params.prop.temp_derate)*totEnergyRequiredByBatt
energyLossPercentage = energyLost/totEnergyRequiredByBatt * 100

% Delivarable 5 shit:
pieChart_vals = [params.performance.W_pay,totBatteryWeight,Weight - params.performance.W_pay - totBatteryWeight];
figure('Name','Payload, Battery, Vehicle Weight Pie Chart');
piechart(pieChart_vals,["Payload Weight","Battery Weight", "Empty Weight"])
% calculate the energy margin:
energyMargin = totBatteryEnergy / totEnergyRequiredByPlane

params.performance.MTOM = Weight;
params.performance.MTOW = Weight * params.env.g;
end