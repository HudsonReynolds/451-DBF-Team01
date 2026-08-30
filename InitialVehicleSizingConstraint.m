% This script does initial sizing of the vehicle based on constraint
% analysis for the system.

% system parameters are all pulled from an Excel spreadsheet. See
% 'SizingParams.xlsx' for more details:

% load the values from the excel sheet
params = readcell("SizingParams.xlsx");

N = length(params);

% put the data into a MATLAB 
for idx = 2:N
    data.(params{idx,1}) = params{idx,2}
end


%% First Constraint: Stall Speed
% this determines the range of values to consider for W/S:
W_S_max = 0.5*data.rho*data.V_S^2*data.CL_max; 
W_S = 0:W_S_max;

%% Second Constraint: Cruise Speed
W_P_cruise = (data.eta_p*data.phi_C / (0.5*data.CD*data.rho*data.V_C^3)) * W_S;

%% Third Constraint: Climb Requirement
% climb at 0.866 L/D max:
L_D_climb = 0.866*data.L_D_max;
W_P_climb = data.eta_p / (data.V_S * (1 / L_D_climb + sin((pi/180) * data.gamma)));

%% Fourth Constraint: Maneuver Requirement
q_m = 0.5 * data.rho * data.V_M^2;
W_P_m = data.eta_p ./ (q_m*data.V_M*(data.CD_0./W_S + data.K*(data.n/q_m)^2*W_S));

%% Fifth Constaint: Takeoff Requirement
numer = 1 - exp(0.6*data.rho*data.g*data.CD_G*data.S_TO*(1./W_S));
denom = data.mu_TO-(data.mu_TO+data.CD_G/data.CL_R) * (exp(0.6*data.rho*data.g*data.CD_G*data.S_TO*1./W_S));
W_P_TO = (numer ./ denom) * (data.eta_p_TO / data.V_TO);

% Final constraint plot:
figure();
boundaryline(W_S_max*ones(2,1),[0;0.25], 'r', 'LineWidth', 1.5, 'DisplayName', 'Stall Speed');
hold on;
boundaryline([0;W_S_max],W_P_climb*ones(2,1), 'g', 'LineWidth', 1.5,'FlipBoundary','on', 'DisplayName', 'Climb Constraint');
boundaryline(W_S,W_P_cruise, 'b', 'LineWidth', 1.5,'FlipBoundary','on', 'DisplayName', 'Cruise Constraint');
boundaryline(W_S,W_P_m, 'c', 'LineWidth', 1.5,'FlipBoundary','on', 'DisplayName', 'Manuever Constraint');
boundaryline(W_S,W_P_TO, 'k', 'LineWidth', 1.5,'FlipBoundary','on', 'DisplayName', 'Takeoff Constraint');
ylim([0, 0.55]);

xlabel("$\frac{W}{S} \left[\frac{N}{m^2}\right]$")
ylabel("$\frac{W}{P} \left[\frac{N}{W}\right]$", 'Rotation', 0)
title('Sizing for Cruise Speed')
legend('Location','northeast')








