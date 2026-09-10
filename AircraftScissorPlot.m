function data = AircraftScissorPlot(data)

% basic script for computing a scissor plot of the aircraft:

winglength = 1.5;
S_w = winglength*data.c_w;
AR_wing = winglength/data.c_w;
x_ac = 0.25;
CM_0 = -0.08;

data.CL_Alpha_Tail = CalcLiftSlope(data.AR_tail, 6.29);
data.CL_Alpha_Wing = CalcLiftSlope(AR_wing);

% Downwash Gradient
kappa = 2;
de_da = kappa* data.CL_Alpha_Wing / (pi * AR_wing);

% compute the total aircraft lift slope:
data.CL_Alpha = data.CL_Alpha_Wing + data.CL_Alpha_Tail*(1-de_da);

x_cg = linspace(0.,0.5);


% Calculation of the Neutral Point:

%x_n = x_ac + ((CL_Alpha_Tail) * (1 - de_da) * V_H) / (CL_Alpha_Wing + S_t / S_w * CL_Alpha_Tail * (1 - de_da));

% Scissor Plot - forward CG Limit, aft CG limit (stability)

% TODO: Update this value once we know control authority limits
CLNoseUp_Tail = -0.7;
CM_EquivalentRotate = 0.1;

% Takeoff Rotation
St_S_takeoff = (CM_0 + data.CL_R .* (x_cg - x_ac) - CM_EquivalentRotate) ./...
    (CLNoseUp_Tail .* ((data.l_t / data.c_w) - x_cg + x_ac));


% Static Margin
SM = .15;

% Stall Recovery
CM_requiredRecovery = -data.CL_max*SM + CM_0;

alpha_stall = deg2rad(15.8);

CL_NoseDown_Tail = data.CL_Alpha_Tail*alpha_stall;

St_S_stall = (CM_0 + data.CL_max .* (x_cg - x_ac) - CM_requiredRecovery) ./...
    (CL_NoseDown_Tail * ((data.l_t / data.c_w) - x_cg + x_ac));


% static margin:
SM = 0.15;

% stability limit:

St_S_stability = (x_cg - x_ac + SM) ./ ((1-de_da)*data.l_t/data.c_w - (x_cg - x_ac + SM));


% TODO: design parameters from this (EXCEL)?
data.St_S = 0.28;
data.x_cg_design = 0.2;
data.SM = SM;

% TODO: choose the chord of the tail:
data.V_H = data.St_S*data.l_t/data.c_t;

figure;
plot(x_cg, St_S_takeoff, 'DisplayName', 'Forward Limit (Takeoff)')
hold on
plot(x_cg, St_S_stall, 'DisplayName', 'Stall Limit')
plot(x_cg,St_S_stability,'DisplayName','Stability Limit')
xlabel('$\bar{x}_{cg}$');
ylabel('$\frac{Sh}{S}$');
yline(0.28, 'b--', 'DisplayName', 'Chosen Wing to Tail Ratio')
title('Aircraft Tail Scissor Plot');
legend('Location','northwest')

end
