function params = VehicleWeightEstimation(params)
% Mission requires 3 full laps (Team01_Requirements.slreqx R3 / Team01_
% StakeholderNeeds.slreqx N2), not 1 -- T_LF and T_TU below used to total
% only ONE lap's worth of straight/turning time (2 straights = T_LF, 2
% turns combining to one full circle = T_TU) and were never multiplied
% up for the other 2 required laps, silently undersizing the battery
% weight fraction (and therefore MTOM/MTOW, which this function sets for
% the whole downstream pipeline) relative to the real mission. Fixed by
% scaling both by the lap count directly, since nothing else in this
% file reads T_LF/T_TU as "one lap" specifically -- they only feed
% LevelFlightBatteryWeightFraction and TurningBatteryWeightFraction below.
N_LAPS = 3;
T_LF = N_LAPS*(2*params.performance.lap_length)/params.performance.V_C;
T_TU = N_LAPS*(2*pi*params.performance.R_req)/params.performance.V_M;

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
% [~,idx] = min(abs(W_batt_payload-W_We));
% Weight = W(idx);
% Weight_y = W_We(idx);
Weight = params.performance.W_pay / (1 - params.performance.W_e_frac - BatteryWeightFraction);
Weight_y = Weight - params.performance.W_e_frac*Weight;
% figure('Name','Total Vehicle Weight Estimate');
% plot(W,W_batt_payload,'g', 'DisplayName', '$W_B + W_P$')
% hold on
% plot(W,W_We,'b', 'DisplayName', '$W - W_e$')
% plot(Weight,Weight_y,'ro','DisplayName',string(Weight))
% xlabel('Total Weight [kg]')
% ylabel('Fractional Weights [kg]')
% title('Weight Estimate')
% legend('Location','Best');

% use total weight for energies
totBatteryWeight = BatteryWeightFraction * Weight;
totBatteryEnergy = totBatteryWeight * params.prop.rho_battery;
totEnergyRequiredByPlane = BatteryWeightFractionPlane * Weight * params.prop.rho_battery;
totEnergyRequiredByBatt = totEnergyRequiredByPlane / (params.prop.eta_p_C*params.prop.eta_m);
energyLost = (1-params.prop.useableCapacity*params.prop.temp_derate)*totEnergyRequiredByBatt;
energyLossPercentage = energyLost/totEnergyRequiredByBatt * 100;

% Delivarable 5 shit:
% pieChart_vals = [params.performance.W_pay,totBatteryWeight,Weight - params.performance.W_pay - totBatteryWeight];
% figure('Name','Payload, Battery, Vehicle Weight Pie Chart');
% piechart(pieChart_vals,["Payload Weight","Battery Weight", "Empty Weight"])
% calculate the energy margin:
energyMargin = totBatteryEnergy / totEnergyRequiredByPlane;

params.performance.MTOM = Weight;
params.performance.MTOW = Weight * params.env.g;

% Exposed for A9 (endurance/range/mission-energy deliverables need the
% actual battery energy budget, not just MTOM): totBatteryEnergy is the
% full pack's energy; usableBatteryEnergy is what's actually available
% to spend in flight once the useableCapacity and temp_derate margins
% already folded into BatteryWeightFraction above are backed out -- by
% construction this equals totEnergyRequiredByPlane exactly.
params.performance.totBatteryMass_kg = totBatteryWeight;
params.performance.totBatteryEnergy_J = totBatteryEnergy;
params.performance.usableBatteryEnergy_J = totEnergyRequiredByPlane;
end