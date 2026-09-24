function params = AircraftScissorPlot(params)

% Basic script for computing a scissor plot of the aircraft:


% generating these new parameters.
params.aero.CL_Alpha_Tail = CalcLiftSlope(params.geometry.AR_hstab); 
params.aero.CL_Alpha_Wing = CalcLiftSlope(params.geometry.AR, 6.29); 

% Downwash Gradient
params.aero.de_da = params.geometry.kappa* params.aero.CL_Alpha_Wing / (pi * params.geometry.AR);

% compute the total aircraft lift slope:
params.aero.CL_Alpha = params.aero.CL_Alpha_Wing + params.aero.CL_Alpha_Tail*(1-params.aero.de_da);

x_cg = linspace(0.,0.5);

% TODO: Update this value once we know control authority limits

%change actuator limits
CL_delta_e_Tail_fwd = params.aero.CL_Alpha_Tail/pi * (acos(1-2*params.geometry.E) + 2*sqrt(params.geometry.E*(1-params.geometry.E)));
CLNoseUp_Tail = -CL_delta_e_Tail_fwd * deg2rad(params.geometry.delta_e_limit_deg);

% Takeoff Rotation
St_S_takeoff = (params.aero.CM_0_Wing + params.aero.CL_R .* (x_cg - params.geometry.x_ac) - params.aero.CM_EquivalentRotate) ./...
    (CLNoseUp_Tail .* ((params.geometry.l_t / params.geometry.c_wing) - x_cg + params.geometry.x_ac));

% Stall Recovery
CM_requiredRecovery = -params.aero.CL_max*params.geometry.SM + params.aero.CM_0_Wing;

alpha_stall_airfoil = deg2rad(params.aero.Airfoil_Alpha_Stall);

CL_NoseDown_Tail = params.aero.CL_Alpha_Tail*alpha_stall_airfoil;

St_S_stall = (params.aero.CM_0_Wing + params.aero.CL_max .* (x_cg - params.geometry.x_ac) - CM_requiredRecovery) ./...
    (CL_NoseDown_Tail * ((params.geometry.l_t / params.geometry.c_wing) - x_cg + params.geometry.x_ac));

% stability limit:
St_S_stability = (x_cg - params.geometry.x_ac + params.geometry.SM) ./ ((1-params.aero.de_da)*params.geometry.l_t/params.geometry.c_wing - (x_cg - params.geometry.x_ac + params.geometry.SM));
params.geometry.V_H = params.geometry.St_S*params.geometry.l_t/params.geometry.c_wing;

%initialization of parameters
params.geometry.x_cg_fwd = interp1(St_S_takeoff, x_cg, params.geometry.St_S);
params.geometry.x_cg_aft = interp1(St_S_stability, x_cg, params.geometry.St_S);

% Neutral Point Calculation
params.geometry.x_n = params.geometry.x_ac + (params.aero.CL_Alpha_Tail*(1-params.aero.de_da)*params.geometry.V_H) / ...
      (params.aero.CL_Alpha_Wing + params.geometry.St_S*params.aero.CL_Alpha_Tail*(1-params.aero.de_da));

%initialization parameter
params.geometry.x_cg_design = params.geometry.x_n - params.geometry.SM;

figure('Name','Aircraft Scissor Plot');
plot(x_cg, St_S_takeoff, 'DisplayName', 'Forward Limit (Takeoff)')
hold on
plot(x_cg, St_S_stall, 'DisplayName', 'Stall Limit')
plot(x_cg,St_S_stability,'DisplayName','Stability Limit')
xlabel('$\bar{x}_{cg}$');
ylabel('$\frac{Sh}{S}$');
yline(params.geometry.St_S, 'b--', 'DisplayName', 'Chosen Wing to Tail Ratio')
title('Aircraft Tail Scissor Plot');
legend('Location','northwest')

end