function data = TrimAircraft(data)

%% Two Equation Trim:
% Here, we look over a range of values for CL_trim, which gives a range of
% solutions for alpha_trim and delta_eTrim

CL_Trim_range = linspace(-.3,1.5);

% compute elevator deflection lift 
CL_delta_e_Tail = data.CL_Alpha_Tail/pi * ...
    (acos(1-2*data.E) + 2*sqrt(data.E*(1-data.E)));

CL_delta_e = data.St_S * CL_delta_e_Tail;

% compute elevator deflection moment:
CM_delta_e = CL_delta_e_Tail*data.St_S*(data.x_cg_design - data.x_ac) - CL_delta_e_Tail * data.V_H;

% % compute the whole aircraft CM_alpha:
CM_Alpha = -data.CL_Alpha*data.SM;

% compute the whole aircraft CM_0:
CL_t0 = 0;

data.CM_0 = data.CM_ac_w + data.CL_0 * (data.x_cg_design - data.x_ac)...
    - CL_t0 * data.St_S * (data.x_cg_design - data.x_ac);

%CM_AC,W + CL_W * (x_CG - x_AC,W / c_w) - CL_T * St/S * lt / c_w + CL_T * St/S * (x_CG - x_AC,W / c_w)

Alpha_Trim = (data.CM_0*CL_delta_e + CM_delta_e * ...
    (CL_Trim_range - data.CL_0)) ./ ...
    ((data.CL_Alpha * CM_delta_e) - (CL_delta_e * CM_Alpha));
delta_e_trim = -1 * ((data.CM_0*data.CL_Alpha + CM_Alpha * (CL_Trim_range - data.CL_0)) / (data.CL_Alpha * CM_delta_e - CL_delta_e * CM_Alpha));

% draw the figure result:
figure('Name','Aircraft Trim v. Trim CL')
subplot(1,2,1)
plot(CL_Trim_range,rad2deg(Alpha_Trim))
xlabel('Trim $C_L$')
ylabel('$\alpha_{\mathrm{trim}}$ [deg]')

subplot(1,2,2)
plot(CL_Trim_range,rad2deg(delta_e_trim))
hold on
yline(15,'r--','15° Limit')
yline(-15,'r--','-15° Limit')
xlabel('Trim $C_L$')
ylabel('$\delta_{\mathrm{e,trim}}$ [deg]')

%% Trim Drag
% compute the drag polar over a range of lift coefficients:
CL = linspace(-1.5,1.5);

CD_Clean = data.CD_0 + data.K_wing*CL.^2;

% plot the clean drag polar and trim 
figure('Name','Drag Polars')
plot(CL,CD_Clean, 'DisplayName', 'Clean Drag Polar')
xlabel('Lift Coefficient [-]')
ylabel('Drag Coefficient [-]')

hold on

%% Tail Lift Required to Trim ---- Beginning of CLAUDE CODE
% Moment balance about the CG: CM_0 + CL_wing*(x_cg - x_ac) - V_H*CL_tail = 0
% Total CL splits between wing and tail: CL = CL_wing + St_S*CL_tail
% Solving the two together for CL_wing(CL):

K_tail = 1 / (pi*data.e_t*data.AR_hstab);

CL_Tail = (data.CM_ac_w + CL*(data.x_cg_design - data.x_ac)) / data.V_H;

CL_Wing = CL - data.St_S * CL_Tail;

%% The Trimmed Drag Polar
CD_Trim = data.CD_0 + data.K_wing*CL_Wing.^2 + data.St_S*K_tail*CL_Tail.^2;

plot(CL, CD_Trim, '--', 'DisplayName', 'Trimmed Drag Polar')
legend('Location','best')

%% End of Claude Code ----


% %% Tail Lift Required to Trim
% CL_T = (CM_ac_Wing + CL * (x_cg - x_ac_wing)) / V_H;
% 
% %% The Trimmed Drag Polar
% CD_Trim = CD_0 + K * (CL_Wing)^2 + St/S * (CL_Tail)^2 / pi / e_t / AR_Tail;



% %% ---- Sizing The Vertical Tail, Rudder, and Ailerons ----
% 
% %% Vertical-Tail Volume Coefficient
% V_v = S_v * l_v / S_w / b;
% S_v = V_v * S_w * b / l_v;
% h_v = sqrt(AR_v * S_v);
% 
% %% Rudder Geometry from Vertical Tail
% S_r = S_rOverS_v * S_v; % where S_rOverS_v 0.30-0.45, b_r = h_v, Epsilon_r_max = +-25-35 degrees
% 
% %% Crosswind Rudder Authority
% 
% Epsilon_r = -1 * (CnBetaOverCnEpsilonr) * Beta; 

end





