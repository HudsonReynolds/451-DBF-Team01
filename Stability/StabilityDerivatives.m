function outputs = StabilityDerivatives(params)

% ---- Longitudinal: CL_alpha, CM_alpha ----
% params.aero.CL_Alpha (whole-aircraft, 3-D, per rad) is already computed in AircraftScissorPlot.m

% Back out (1 - de/da) from the already-exported CL_Alpha rather than
% re-deriving AR_wing/kappa locally (keeps this in sync with
% AircraftScissorPlot.m automatically instead of duplicating its internals):
one_minus_deda = (params.aero.CL_Alpha - params.aero.CL_Alpha_Wing)/params.aero.CL_Alpha_Tail;

% Neutral point (Deliverable 2 asks for this and it's currently commented
% out in AircraftScissorPlot.m -- computed here instead, same formula):
x_n = params.geometry.x_ac + (params.aero.CL_Alpha_Tail*one_minus_deda*params.geometry.V_H) / ...
      (params.aero.CL_Alpha_Wing + params.geometry.St_S*params.aero.CL_Alpha_Tail*one_minus_deda);

SM_actual = x_n - params.geometry.x_cg_design;
CM_Alpha  = -params.aero.CL_Alpha*SM_actual;

fprintf('--- Longitudinal derivatives (CG = %.2f MAC) ---\n', params.geometry.x_cg_design);
fprintf('  CL_alpha (aircraft, 3-D)  = %.3f /rad\n', params.aero.CL_Alpha);
fprintf('  Neutral point x_n         = %.3f MAC\n', x_n);
fprintf('  Static margin (from x_n)  = %.3f  (design assumption was %.2f -- reconcile)\n', SM_actual, params.geometry.SM);
fprintf('  CM_alpha                  = %.3f /rad\n', CM_Alpha);
if CM_Alpha > 0
    warning('CM_alpha is positive -- CG is aft of the neutral point, statically unstable. Fix the scissor plot inputs before trusting anything downstream.');
end

%% ---- Lateral-directional: Cn_beta, Cl_beta ----
b = params.geometry.S_wing/params.geometry.c_wing; % wingspan [m], rectangular wing

% Vertical tail geometry -- duplicated from ControlSurfaceSizing.m, PLACEHOLDER,
% keep these two files in sync until they're pulled into `data`/Excel. 
AR_v = 1.5;
CL_Alpha_VT = CalcLiftSlope(AR_v, 6.29);

Cn_beta_VT = CL_Alpha_VT*params.geometry.V_v; % vertical-tail-alone contribution
Cn_beta    = Cn_beta_VT;      % fuselage/wing terms neglected -- PLACEHOLDER, see note above

Cn_beta_stable_range = [0.05, 0.15]; % /rad, typical light GA/RC aircraft -- verify against Sadraey's tables

fprintf('--- Lateral-directional derivatives ---\n');
fprintf('  Cn_beta (VT only) = %.4f /rad', Cn_beta);
if Cn_beta >= Cn_beta_stable_range(1) && Cn_beta <= Cn_beta_stable_range(2)
    fprintf('  [within typical stable range %.2f-%.2f]\n', Cn_beta_stable_range(1), Cn_beta_stable_range(2));
else
    fprintf('  [OUTSIDE typical stable range %.2f-%.2f -- reconcile]\n', Cn_beta_stable_range(1), Cn_beta_stable_range(2));
end

% Dihedral effect on Cl_beta, via the same strip-theory approach used for
% roll damping/roll control power in ControlSurfaceSizing.m -- shown as an
% integral (not a memorized coefficient) so it's auditable:
Gamma_dihedral_deg = 4;

Gamma_dihedral = deg2rad(Gamma_dihedral_deg);

y = linspace(0, b/2, 400);
c_of_y = params.geometry.c_wing*ones(size(y)); % rectangular wing, no taper

Cl_beta_dihedral = -0.5 * Gamma_dihedral * params.aero.CL_Alpha_Wing;


% Initial Claude Initialization
% 2*Gamma_dihedral*params.aero.CL_Alpha_Wing/(data.S_wing*b) * trapz(y,c_of_y.*y);


Cl_beta = (-1.2 * sqrt(params.geometry.AR) * params.geometry.zw / params.geometry.wingspan * 2 * params.geometry.df / params.geometry.wingspan) + Cl_beta_dihedral;

Cl_beta_stable_range = [-0.20, -0.05]; % /rad, typical light GA/RC aircraft -- verify against Sadraey's tables

fprintf('  Dihedral = %.1f deg\n', Gamma_dihedral_deg);
fprintf('  Cl_beta (dihedral only) = %.4f /rad (vs. %.4f /rad at zero dihedral)', Cl_beta_dihedral, Cl_beta);
if Cl_beta_dihedral <= Cl_beta_stable_range(2) && Cl_beta_dihedral >= Cl_beta_stable_range(1)
    fprintf('  [within typical stable range %.2f to %.2f]\n', Cl_beta_stable_range(1), Cl_beta_stable_range(2));
else
    fprintf('  [OUTSIDE typical stable range %.2f to %.2f -- reconcile]\n', Cl_beta_stable_range(1), Cl_beta_stable_range(2));
end

outputs.StabilityDerivatives.CL_Alpha  = params.aero.CL_Alpha;
outputs.StabilityDerivatives.x_n       = x_n;
outputs.StabilityDerivatives.SM        = SM_actual;
outputs.StabilityDerivatives.CM_Alpha  = CM_Alpha;
outputs.StabilityDerivatives.Cn_beta   = Cn_beta;
outputs.StabilityDerivatives.Gamma_deg = Gamma_dihedral_deg;
outputs.StabilityDerivatives.Cl_beta   = Cl_beta;

end
