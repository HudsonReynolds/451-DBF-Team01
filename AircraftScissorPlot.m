% basic script for computing a scissor plot of the aircraft:
AR_tail = 3.5;
AR_wing = 7.2;
c_w = 0.3909;
S_w = 1.1;

% UPDATE CALC LIFT SLOPE FOR HELMBOLD

CL_alpha_tail = CalcLiftSlope(AR_tail);

CL_alpha_wing = CalcLiftSlope(AR_wing);

%
de_da = 2* CL_alpha_wing / (pi * AR_wing);

x_cg = linspace(0,0.6);


% moment coefficient about aerodynamic center wing (NEED VALUE FROM XFLR)
CM_ac_w = -.103;

% wing and tail lift coefficients (NEED VALUE):
CL_W_R = 1.405;
CL_T_R = -1.031;

% CM_cg
CM_cg_R = 0.015;

% static margin:
SM = 0.1;

% aerodynamic center for the wing (x/c):
x_ac_w = .25;

% length between AC of wing and AC of tail [m]
lt = 1.5; %[m]


Sh_S_aft = CL_alpha_wing.*(x_cg - x_ac_w + SM) ./ ...
    (CL_alpha_tail .* (1-de_da)*lt./c_w - x_cg + x_ac_w - SM);


Sh_S_SM_2 = CL_alpha_wing.*(x_cg - x_ac_w + 0.2) ./ ...
    (CL_alpha_tail .* (1-de_da)*lt./c_w - x_cg + x_ac_w - 0.2);

Sh_S_for = (CM_ac_w + CL_W_R.*(x_cg - x_ac_w) - CM_cg_R)./ ...
    (CL_T_R .* (lt./c_w - x_ac_w + 1/4));


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


function slope = CalcLiftSlope(AR)
    slope = pi*AR / (1+sqrt(1+(AR/2)^2));
end