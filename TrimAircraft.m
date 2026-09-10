function TrimAircraft(data)
%% ---- Trim & Trim Drag Equations ----

%% The Trim Condition
CM = 0;
data.V_C
CL_Trim = 2*data.W / data.rho / data.V_C^2 / data.S_wing;
CL = CL_Trim;


%% Two Equation Trim:

% Here, we look over a range of values for CL_trim, which gives a range of
% solutions for alpha_trim and delta_eTrim

CL_Trim_range = linspace(0,1);

% compute elevator deflection lift 
CL_delta_e_Tail = data.CL_Alpha_Tail/pi * ...
    (acos(1-2*data.E) + 2*sqrt(data.E*(1-data.E)));

CL_delta_e = data.St_S * CL_delta_e_Tail;

% compute elevator deflection moment:
CM_delta_e = CL_delta_e_Tail*data.St_S*(data.x_cg_design - data.x_ac) - CL_delta_e_Tail * data.V_H;

Alpha_Trim = (data.CM_0*CL_delta_e + CM_delta_e * ...
    (CL_Trim_range - data.CL_0)) ./ ...
    ((data.CL_Alpha * CM_delta_e) - (CL_delta_e * CM_Alpha));
Epsilon_e_trim = -1 * ((CM0*CL_Alpha + CM_Alpha * (CL_trim - CL_0)) / (CL_Alpha * CM_Epsilon_e - CL_Epsilon_e * CM_Alpha));



%% Trim Drag
CD_Clean = CD_0 + K*CL^2;

%% Tail Lift Required to Trim
CL_T = (CM_ac_Wing + CL * (x_cg - x_ac_wing)) / V_H;

%% The Trimmed Drag Polar
CD_Trim = CD_0 + K * (CL_Wing)^2 + St/S * (CL_Tail)^2 / pi / e_t / AR_Tail;

%% ---- Sizing The Vertical Tail, Rudder, and Ailerons ----

%% Vertical-Tail Volume Coefficient
V_v = S_v * l_v / S_w / b;
S_v = V_v * S_w * b / l_v;
h_v = sqrt(AR_v * S_v);

%% Rudder Geometry from Vertical Tail
S_r = S_rOverS_v * S_v; % where S_rOverS_v 0.30-0.45, b_r = h_v, Epsilon_r_max = +-25-35 degrees

%% Crosswind Rudder Authority

Epsilon_r = -1 * (CnBetaOverCnEpsilonr) * Beta; 

end





