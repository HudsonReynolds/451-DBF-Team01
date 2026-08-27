% This script does initial sizing of the vehicle based on constraint
% analysis for the system.

% --- parameters ---

% these will need to be justified in our document and edited based off
% those decisions:

% stall speed less than 11 meters per second

CL_max = 1.2; % [-], max lift coefficient
L_D = 7.332;  % [-], lift to drag ratio
V_s = 10;     % [m⋅s⁻¹], stall speed
AR = 6;       % [-], aspect ratio
e = 0.7;      % [-], spanwise efficiency
eta_p = 0.6;  % [-], propeller efficiency
Cd0 = 0.04;   % [-], whole aircraft

% known quantities:
rho = 1.225;  % [kg⋅m⁻³], air density
gamma = 5;    % [deg], flight path angle in climb

%% First Constraint: Stall Speed
% this determines the range of values to consider for W/S:
W_S_max = 0.5*rho*V_s^2*CL_max; 

W_S = 0:W_S_max;

%% Second Constraint: Cruise Speed
Phi = 0.75; % throttle setting

V_c = 1.7*V_s;

W_P_cruise = (eta_p*Phi / (0.5*1.1*Cd0*rho*V_c^3)) * W_S;

% plot the result:
figure();
boundaryline(W_S,W_P_cruise, 'r', 'LineWidth', 1.5,'HatchAngle',80, 'HatchLength',0.02);
xlabel("$\frac{W}{S} \left[\frac{N}{m^2}\right]$")
ylabel("$\frac{W}{P} \left[\frac{N}{W}\right]$", 'Rotation', 0)
title('Sizing for Cruise Speed')


%% Third Constraint: Climb Requirement

% climb at 0.866 L/D max:
L_D_climb = 0.866*L_D;

%W_P_climb = eta_p / 

%% Fourth Constraint: Maneuver Requirement

%% Fifth Constaint: Takeoff Requirement






