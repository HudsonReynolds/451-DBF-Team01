function AircraftScissorPlot(data)

% basic script for computing a scissor plot of the aircraft:


% length between AC of wing and AC of tail [m]
l_t = 0.55; %[m]

AR_tail = 3;
winglength = 1.5;
c_w = 0.291;
S_w = winglength*c_w;
AR_wing = winglength/c_w;
x_ac = 0.25;
CM_0 = -0.08;

CL_Alpha_Tail = CalcLiftSlope(AR_tail, 6.29);

CL_Alpha_Wing = CalcLiftSlope(AR_wing);

% Downwash Gradient
kappa = 2;
de_da = kappa* CL_Alpha_Wing / (pi * AR_wing);

x_cg = linspace(0.,0.5);


% Calculation of the Neutral Point:

%x_n = x_ac + ((CL_Alpha_Tail) * (1 - de_da) * V_H) / (CL_Alpha_Wing + S_t / S_w * CL_Alpha_Tail * (1 - de_da));

% Scissor Plot - forward CG Limit, aft CG limit (stability)

% TODO: Update this value once we know control authority limits
CLNoseUp_Tail = -0.7;
CM_EquivalentRotate = 0.1;

% Takeoff Rotation
St_S_takeoff = (CM_0 + data.CL_R .* (x_cg - x_ac) - CM_EquivalentRotate) ./...
    (CLNoseUp_Tail .* ((l_t / c_w) - x_cg + x_ac));



% Static Margin
SM = .15;

% Stall Recovery
CM_requiredRecovery = -data.CL_max*SM + CM_0;

alpha_stall = deg2rad(15.8);

CL_NoseDown_Tail = CL_Alpha_Tail*alpha_stall;

St_S_stall = (CM_0 + data.CL_max .* (x_cg - x_ac) - CM_requiredRecovery) ./...
    (CL_NoseDown_Tail * ((l_t / c_w) - x_cg + x_ac));


% wing and tail lift coefficients (NEED VALUE):
CL_W_R = 1.405;
CL_T_R = -1.031;

% CM_cg
CM_cg_R = 0.015;

% static margin:
SM = 0.15;

% stability limit:

St_S_stability = (x_cg - x_ac + SM) ./ ((1-de_da)*l_t/c_w - (x_cg - x_ac + SM));





figure;
plot(x_cg, St_S_takeoff, 'DisplayName', 'Forward Limit (Takeoff)')
hold on
plot(x_cg, St_S_stall, 'DisplayName', 'Stall Limit')
plot(x_cg,St_S_stability,'DisplayName','Stability Limit')
xlabel('$\bar{x}_{cg}$');
ylabel('$\frac{Sh}{S}$');
yline(0.3, 'b--', 'DisplayName', 'Chosen Wing to Tail Ratio')
title('Aircraft Tail Scissor Plot');
legend('Location','northwest')

end
