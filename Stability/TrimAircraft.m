function params = TrimAircraft(params)

%% Two Equation Trim:
% Here, we look over a range of values for CL_trim, which gives a range of
% solutions for alpha_trim and delta_eTrim

CL_Trim_range = linspace(-.3,1.5);

% compute elevator deflection lift 
CL_delta_e_Tail = params.aero.CL_Alpha_Tail/pi * ...
    (acos(1-2*params.geometry.E) + 2*sqrt(params.geometry.E*(1-params.geometry.E)));

CL_delta_e = params.geometry.St_S * CL_delta_e_Tail;

% compute elevator deflection moment:
CM_delta_e = CL_delta_e_Tail*params.geometry.St_S*(params.geometry.x_cg_design - params.geometry.x_ac) - CL_delta_e_Tail * params.geometry.V_H;

% % compute the whole aircraft CM_alpha:
CM_Alpha = -params.aero.CL_Alpha*params.geometry.SM;

% compute the whole aircraft CM_0:
CL_t0 = 0;
params.aero.CM_0 = params.aero.CM_ac_w + params.aero.CL_0 * (params.geometry.x_cg_design - params.geometry.x_ac)...
    - CL_t0 * params.geometry.St_S * (params.geometry.x_cg_design - params.geometry.x_ac);

Alpha_Trim = (params.aero.CM_0*CL_delta_e + CM_delta_e * ...
    (CL_Trim_range - params.aero.CL_0)) ./ ...
    ((params.aero.CL_Alpha * CM_delta_e) - (CL_delta_e * CM_Alpha));
delta_e_trim = -1 * ((params.aero.CM_0*params.aero.CL_Alpha + CM_Alpha * (CL_Trim_range - params.aero.CL_0)) / (params.aero.CL_Alpha * CM_delta_e - CL_delta_e * CM_Alpha));

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
CL = linspace(0, params.aero.CL_max);
CD_clean = params.aero.CD_0 + params.aero.K_wing*CL.^2;

% plot the clean drag polar and trim 
figure('Name','Drag Polars')
plot(CD_clean, CL, 'DisplayName', 'Clean Drag Polar')
xlabel('$C_D$')
ylabel('$C_L$')
title('Drag Polar')
hold on

%% Tail Lift Required to Trim
CL_hstab = (params.aero.CM_ac_w + CL*(params.geometry.x_cg_design - params.geometry.x_ac)) / params.geometry.V_H;
CL_wing = CL - params.geometry.St_S * CL_hstab;

%% The Trimmed Drag Polar
CD_trim = params.aero.CD_0 + params.aero.K_wing*CL_wing.^2 + params.geometry.St_S*params.aero.K_hstab*CL_hstab.^2;

plot(CD_trim, CL, '--', 'DisplayName', 'Trimmed Drag Polar')
legend('Location','best')
end





