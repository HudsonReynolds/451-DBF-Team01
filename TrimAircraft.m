function TrimAircraft(data)
%% ---- Trim & Trim Drag Equations ----

%% The Trim Condition
CM = 0;
CL_Trim = 2*data.W / data.rho / V^2 / S_w;
CL = CL_Trim;

%% Two Equation Trim

Alpha_Trim = (data.CM_0*CL_Epsilon_e + CM_Epsilon_e * (CL_Trim - CL_0)) / ((CL_Alpha * CM_Epsilon_e) - (CL_Epsilon_e * CM_Alpha));
Epsilon_e_trim = -1 * ((CM0*CL_Alpha + CM_Alpha * (CL_trim - CL_0)) / (CL_Alpha * CM_Epsilon_e - CL_Epsilon_e * CM_Alpha));


%% Elevator Derivatives
CL_Epsilon_e = S_t * CL_Epsilon_e_Tail / S_w;
CM_Epsilon_e = CL_Epsilon_e_Tail * S_t * (x_cg - x_ac_wing) / S_w - CL_Epsilon_e_Tail * V_H;

E = C_e / C_t;
CL_Epsilon_e_Tail = CL_Alpha_Tail / pi * (acos(1-2*E) + 2*sqrt(E*(1-E)));

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





