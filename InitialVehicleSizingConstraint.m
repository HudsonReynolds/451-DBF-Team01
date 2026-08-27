% This script does initial sizing of the vehicle based on constraint
% analysis for the system.

% --- parameters ---

% these will need to be justified in our document and edited based off
% those decisions:

CL_max = 1.2; % [-]
L_D = 12;     % [-]
V_s = 8;      % [m⋅s⁻¹]
AR = 6;       % [-]
e = 0.7;      % [-]
eta_p = 0.6;  % [-]
Cd0 = 0.05;   % [-]

% known quantities:
rho = 1.225;  % [kg⋅m⁻³]

%% First Constraint: Stall Speed
% this determines the range of values to consider for W/S:
W_S_max = 0.5*rho*V_s^2*CL_max; 

W_S = 0:W_S_max;

%% Second Constraint: Cruise Speed
Phi = 0.75; % throttle setting

V_c = 1.7*V_s;

W_P = (eta_p*Phi / (0.5*1.1*Cd0*rho*V_c^3)) * W_S;

% plot the result:
figure();
plot(W_S,W_P)
xlabel("$\frac{W}{S} \left[\frac{N}{m^2}\right]$")
ylabel("$\frac{W}{P} \left[\frac{N}{W}\right]$", 'Rotation', 0)
title('Sizing for Cruise Speed')



%% Third Constraint: Climb Requirement

%% Fourth Constraint: Maneuver Requirement

%% Fifth Constaint: Takeoff Requirement






