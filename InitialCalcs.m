function params = InitialCalcs(params)

%% Speeds and flight conditions
params.performance.V_C = sqrt(2*params.performance.W_S_design / (params.env.rho*params.aero.CL_C)); % Cruise speed from design CL_C [m/s]
params.performance.V_TO = params.performance.mult_V_TO * params.performance.V_S; % Takeoff speed [m/s]
params.performance.V_M = sqrt(params.performance.R_M * params.env.g * sqrt(params.performance.n^2 - 1)); % Maneuver speed for turn radius R_M [m/s]
params.performance.V_S_M = params.performance.V_S * sqrt(params.performance.n); % Stall speed at load factor n [m/s]
params.performance.bank_angle = acosd(1 / params.performance.n); % Bank angle at load factor n [deg]
params.performance.gamma = atand(params.performance.climb_alt / params.performance.climb_dist); % Climb angle [deg]

%% Lift coefficients
params.aero.CL_TO = params.aero.CL_C; % Takeoff CL before rotation, no flaps, same as cruise [-]
params.aero.CL_R = params.aero.CL_max / params.performance.mult_V_TO^2; % Rotation CL [-]

%% Weight and wing
params.performance.MTOW = params.env.g * params.performance.MTOM; % Takeoff weight [N]
params.geometry.S_wing = params.performance.MTOW / params.performance.W_S_design; % Wing area from design wing loading [m^2]
params.geometry.c_wing = params.geometry.S_wing / (params.geometry.wingspan - params.geometry.width_fuselage); % Wing chord, area excludes fuselage section [m]
params.geometry.AR = params.geometry.wingspan / params.geometry.c_wing; % Wing aspect ratio [-]
params.aero.e_wing = 1.78*(1 - 0.045*params.geometry.AR^0.68) - 0.64; % Oswald efficiency, Sadraey eqn 5.27a [-]
params.aero.K_wing = 1 / (pi * params.aero.e_wing * params.geometry.AR); % Induced drag factor [-]

%% Horizontal tail
params.geometry.S_hstab = params.geometry.St_S * params.geometry.S_wing; % H-stab area from area ratio [m^2]
params.geometry.span_hstab = sqrt(params.geometry.S_hstab * params.geometry.AR_hstab); % H-stab span [m]
params.geometry.c_hstab = sqrt(params.geometry.S_hstab / params.geometry.AR_hstab); % H-stab chord [m]
params.geometry.V_H = params.geometry.St_S * params.geometry.l_t / params.geometry.c_wing; % Horizontal tail volume coefficient [-]
params.aero.K_hstab = 1 / (pi * params.aero.e_t * params.geometry.AR_hstab); % H-stab induced drag factor [-]

%% Vertical tail
params.geometry.S_v = params.geometry.V_v * params.geometry.S_wing * params.geometry.wingspan / params.geometry.l_t; % Total fin area from volume coefficient, fin arm = l_t [m^2]
params.geometry.S_fin = params.geometry.S_v / params.geometry.n_vfins; % Area per fin [m^2]
params.geometry.span_vstab = sqrt(params.geometry.S_fin * params.geometry.AR_vstab); % Fin span [m]
params.geometry.c_vstab = sqrt(params.geometry.S_fin / params.geometry.AR_vstab); % Fin chord [m]

end