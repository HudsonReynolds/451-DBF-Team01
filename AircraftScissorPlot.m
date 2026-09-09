% basic script for computing a scissor plot of the aircraft:

% length between AC of wing and AC of tail [m]
l_t = 0.4; %[m]

AR_tail = 3;
AR_wing = 7.2;
c_w = 0.3909;
S_w = 1.1;
x_ac = 0.25;

CL_Alpha_Tail = CalcLiftSlope(AR_tail);

CL_Alpha_Wing = CalcLiftSlope(AR_wing);

% Downwash Gradient
kappa = 2;
de_da = kappa* CL_Alpha_Wing / (pi * AR_wing);

x_cg = linspace(0,0.6);


% Calculation of the Neutral Point:

x_n = x_ac + ((CL_Alpha_Tail) * (1 - de_da) * V_H) / (CL_Alpha_Wing + S_t / S_w * CL_Alpha_Tail * (1 - de_da));

% Scissor Plot - forward CG Limit, aft CG limit (stability)

% Takeoff Rotation
St_S_takeoff = (CMO_Wing + CL_Rot * (x_cg - x_ac) - CM_EquivalentRotate) /...
    (CLNoseUp_Tail * ((l_t / c) - x_cg + x_ac));



% Static Margin
SM = .15;
x_cg = x_n - SM;

% Stall Recovery
St_S_stall = (CMO_Wing + CL_Max * (x_cg - x_ac) - CM_requiredRecovery) /...
    (CL_NoseDown_Tail * ((l_t / c) - x_cg + x_ac));




% moment coefficient about aerodynamic center wing (NEED VALUE FROM XFLR)
CM_ac_w = -.082;

% wing and tail lift coefficients (NEED VALUE):
CL_W_R = 1.405;
CL_T_R = -1.031;

% CM_cg
CM_cg_R = 0.015;

% static margin:
SM = 0.1;

% aerodynamic center for the wing (x/c):
x_ac_w = .25;




Sh_S_aft = CL_Alpha_Wing.*(x_cg - x_ac_w + SM) ./ ...
    (CL_Alpha_Tail .* (1-de_da)*l_t./c_w - x_cg + x_ac_w - SM);


Sh_S_SM_2 = CL_Alpha_Wing.*(x_cg - x_ac_w + 0.2) ./ ...
    (CL_Alpha_Tail .* (1-de_da)*l_t./c_w - x_cg + x_ac_w - 0.2);

Sh_S_for = (CM_ac_w + CL_W_R.*(x_cg - x_ac_w) - CM_cg_R)./ ...
    (CL_T_R .* (l_t./c_w - x_ac_w + 1/4));


figure;
plot(x_cg, Sh_S_for, 'DisplayName', 'Forward Limit')
hold on
plot(x_cg, Sh_S_aft, 'DisplayName', 'Aft Limit $SM = 0.1$')
plot(x_cg, Sh_S_SM_2, 'DisplayName', 'Selected $\frac{Sh}{S} = 0.1$')
xlabel('$\bar{x}_{cg}$');
ylabel('$\frac{Sh}{S}$');
yline(0.1, 'b--', 'DisplayName', 'Chosen Wing to Tail Ratio')
title('Aircraft Tail Scissor Plot');
legend('Location','northwest')
