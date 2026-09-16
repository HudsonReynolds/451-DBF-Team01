function params = AircraftScissorPlot(params)

% basic script for computing a scissor plot of the aircraft:

winglength = 1.5;
AR_wing = winglength/params.geometry.c_wing;
x_ac = 0.25;
CM_0 = -0.08;

% generating these new parameters.
params.aero.CL_Alpha_Tail = CalcLiftSlope(params.geometry.AR_hstab, 6.29); 
params.aero.CL_Alpha_Wing = CalcLiftSlope(AR_wing); 

% Downwash Gradient
kappa = 2;
de_da = kappa* params.aero.CL_Alpha_Wing / (pi * AR_wing);

% generating these new parameters
% compute the total aircraft lift slope:
params.aero.CL_Alpha = params.aero.CL_Alpha_Wing + params.aero.CL_Alpha_Tail*(1-de_da);

x_cg = linspace(0.,0.5);

% Calculation of the Neutral Point:

%x_n = x_ac + ((CL_Alpha_Tail) * (1 - de_da) * V_H) / (CL_Alpha_Wing + S_t / S_w * CL_Alpha_Tail * (1 - de_da));

% Scissor Plot - forward CG Limit, aft CG limit (stability)

% TODO: Update this value once we know control authority limits

%change actuator limits
params.geometry.delta_e_limit_deg = 20; % [deg] elevator travel limit -- confirm against servo/horn travel
CL_delta_e_Tail_fwd = params.aero.CL_Alpha_Tail/pi * (acos(1-2*params.geometry.E) + 2*sqrt(params.geometry.E*(1-params.geometry.E)));
CLNoseUp_Tail = -CL_delta_e_Tail_fwd * deg2rad(params.geometry.delta_e_limit_deg);
CM_EquivalentRotate = 0.1;

% Takeoff Rotation
St_S_takeoff = (CM_0 + params.aero.CL_R .* (x_cg - x_ac) - CM_EquivalentRotate) ./...
    (CLNoseUp_Tail .* ((params.geometry.l_t / params.geometry.c_wing) - x_cg + x_ac));

% Stall Recovery
CM_requiredRecovery = -params.aero.CL_max*params.geometry.SM + CM_0;

alpha_stall = deg2rad(15.8);

CL_NoseDown_Tail = params.aero.CL_Alpha_Tail*alpha_stall;

St_S_stall = (CM_0 + params.aero.CL_max .* (x_cg - x_ac) - CM_requiredRecovery) ./...
    (CL_NoseDown_Tail * ((params.geometry.l_t / params.geometry.c_wing) - x_cg + x_ac));

% stability limit:
St_S_stability = (x_cg - x_ac + params.geometry.SM) ./ ((1-de_da)*params.geometry.l_t/params.geometry.c_wing - (x_cg - x_ac + params.geometry.SM));

% TODO: choose the chord of the tail:
data.V_H = params.geometry.St_S*params.geometry.l_t/params.geometry.c_wing;

%initialization of parameters
params.geometry.x_cg_fwd = interp1(St_S_takeoff, x_cg, params.geometry.St_S);
params.geometry.data.x_cg_aft = interp1(St_S_stability, x_cg, params.geometry.St_S);

% Neutral Point Calculation
one_minus_deda = (params.aero.CL_Alpha - params.aero.CL_Alpha_Wing)/params.aero.CL_Alpha_Tail;
x_n = data.x_ac + (params.aero.CL_Alpha_Tail*one_minus_deda*data.V_H) / ...
      (params.aero.CL_Alpha_Wing + params.geometry.St_S*params.aero.CL_Alpha_Tail*one_minus_deda);

%initialization parameter
params.geometry.x_cg_design = x_n - params.geometry.SM;

figure('Name','Aircraft Scissor Plot');
plot(x_cg, St_S_takeoff, 'DisplayName', 'Forward Limit (Takeoff)')
hold on
plot(x_cg, St_S_stall, 'DisplayName', 'Stall Limit')
plot(x_cg,St_S_stability,'DisplayName','Stability Limit')
xlabel('$\bar{x}_{cg}$');
ylabel('$\frac{Sh}{S}$');
yline(data.St_S, 'b--', 'DisplayName', 'Chosen Wing to Tail Ratio')
title('Aircraft Tail Scissor Plot');
legend('Location','northwest')

end
