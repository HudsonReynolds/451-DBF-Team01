% This script does initial sizing of the vehicle based on constraint
% analysis for the system.

% --- parameters ---

% these will need to be justified in our document and edited based off
% those decisions:

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
Phi = 0.75; % throttle setting

V_c = 1.7*data.V_S;

W_P_cruise = (data.eta_p*data.phi_C / (0.5*1.1*data.CD_0*data.rho*data.V_C^3)) * W_S;

%% Third Constraint: Climb Requirement

% climb at 0.866 L/D max:
L_D_climb = 0.866*data.L_D_max;
W_P_climb = data.eta_p / (data.V_S * (1 / L_D_climb + sin((pi/180) * data.gamma)));

%% Fourth Constraint: Maneuver Requirement
q_m = 0.5 * data.rho * data.V_S_M^2;

% which velocity to use here?
V_m = 25;

W_P_m = data.eta_p ./ (q_m*V_m*((data.CD_0./W_S) + (1/(pi*data.AR*data.e))*(data.n/q_m)^2*W_S));

%% Fifth Constaint: Takeoff Requirement
V_to = 1.1 * V_s;
S_to = 25; % [m], takeoff distance

CL_TO = CL_C;
CD_TO = data.CD_0 + data.K * data.CL_TO;
%CD_G = (CD_TO - mu*CL_TO);

%numer = 1 - exp(0.6*rho*g*CD_G*S_TO*1./(W_S));

%denom = mu-(mu+CD_G/CL_R) * (exp(0.6*rho*g*CD_G*S_TO*1./W_S));


% Final constraint plot:
figure();
boundaryline(W_S,W_P_cruise, 'r', 'LineWidth', 1.5,'FlipBoundary','on', 'DisplayName', 'Cruise Constraint');
hold on;
boundaryline(W_S_max*ones(2,1),[0;0.25], 'b', 'LineWidth', 1.5, 'DisplayName', 'Stall Speed')

boundaryline(W_S,W_P_m, 'g', 'LineWidth', 1.5,'FlipBoundary','on', 'DisplayName', 'Manuver Constraint');

xlabel("$\frac{W}{S} \left[\frac{N}{m^2}\right]$")
ylabel("$\frac{W}{P} \left[\frac{N}{W}\right]$", 'Rotation', 0)
title('Sizing for Cruise Speed')
legend('Location','northwest')








