function params = calcAero(params)

%% Drag coefficients
params.aero.CD_C = params.aero.CD_0 + params.aero.K_wing*params.aero.CL_C^2; % Cruise CD [-]
params.aero.CD_TO = params.aero.CD_0 + params.aero.K_wing*params.aero.CL_TO^2; % Takeoff CD [-]
params.aero.CD_G = params.aero.CD_TO - params.env.mu_TO*params.aero.CL_TO; % Ground roll CD, Sadraey eqn 4.67a [-]

%% Lift-to-drag
params.aero.L_D_C = params.aero.CL_C / params.aero.CD_C; % L/D at cruise [-]
params.aero.L_D_max = 1 / (2*sqrt(params.aero.CD_0*params.aero.K_wing)); % Max L/D of parabolic polar [-]

% CL = linspace(0, params.aero.CL_max);
% CD = CD_0 + params.aero.K_wing.*CL.^2;
% L_D = CL ./ CD;
% L_D_cruise = params.aero.CL_C / (CD_0 + params.aero.K_wing*params.aero.CL_C^2)
% L_D_max = max(L_D)
% CL_max_LD = CL(L_D==L_D_max)

end