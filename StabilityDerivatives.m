function data = StabilityDerivatives(data)
% Deliverable 5 - Longitudinal and lateral-directional stability derivatives
%
% PLACEHOLDER ASSUMPTIONS to reconcile:
%   - Gamma_dihedral_deg : wing dihedral angle -- not yet a design choice anywhere else
%   - V_v, AR_v          : vertical tail geometry -- duplicated from ControlSurfaceSizing.m,
%                           factor these into `data` once finalized so the two files agree
%   - Cn_beta here omits fuselage/wing contributions (usually destabilizing/negative for
%     the fuselage) -- vertical-tail-alone value is a first-pass, not the final number

%% ---- Longitudinal: CL_alpha, CM_alpha ----
% data.CL_Alpha (whole-aircraft, 3-D, per rad) is already computed in AircraftScissorPlot.m

% Back out (1 - de/da) from the already-exported CL_Alpha rather than
% re-deriving AR_wing/kappa locally (keeps this in sync with
% AircraftScissorPlot.m automatically instead of duplicating its internals):
one_minus_deda = (data.CL_Alpha - data.CL_Alpha_Wing)/data.CL_Alpha_Tail;

% Neutral point (Deliverable 2 asks for this and it's currently commented
% out in AircraftScissorPlot.m -- computed here instead, same formula):
x_n = data.x_ac + (data.CL_Alpha_Tail*one_minus_deda*data.V_H) / ...
      (data.CL_Alpha_Wing + data.St_S*data.CL_Alpha_Tail*one_minus_deda);

SM_actual = x_n - data.x_cg_design;
CM_Alpha  = -data.CL_Alpha*SM_actual;

fprintf('--- Longitudinal derivatives (CG = %.2f MAC) ---\n', data.x_cg_design);
fprintf('  CL_alpha (aircraft, 3-D)  = %.3f /rad\n', data.CL_Alpha);
fprintf('  Neutral point x_n         = %.3f MAC\n', x_n);
fprintf('  Static margin (from x_n)  = %.3f  (design assumption was %.2f -- reconcile)\n', SM_actual, data.SM);
fprintf('  CM_alpha                  = %.3f /rad\n', CM_Alpha);
if CM_Alpha > 0
    warning('CM_alpha is positive -- CG is aft of the neutral point, statically unstable. Fix the scissor plot inputs before trusting anything downstream.');
end

%% ---- Lateral-directional: Cn_beta, Cl_beta ----
b = data.S_wing/data.c_w; % wingspan [m], rectangular wing

% Vertical tail geometry -- duplicated from ControlSurfaceSizing.m, PLACEHOLDER,
% keep these two files in sync until they're pulled into `data`/Excel.
V_v  = 0.04;
AR_v = 1.5;
l_v  = data.l_t;
CL_Alpha_VT = CalcLiftSlope(AR_v, 6.29);

Cn_beta_VT = CL_Alpha_VT*V_v; % vertical-tail-alone contribution
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
Gamma_dihedral_deg = 0;
Gamma_dihedral = deg2rad(Gamma_dihedral_deg);

y = linspace(0, b/2, 400);
c_of_y = data.c_w*ones(size(y)); % rectangular wing, no taper

Cl_beta_dihedral    = -2*Gamma_dihedral*data.CL_Alpha_Wing/(data.S_wing*b) * trapz(y, c_of_y.*y);
Cl_beta_no_dihedral = 0; % wing-alone, zero-dihedral baseline for comparison

df = .1; % body depth at the wing
zw = .04; %wing root height

Cl_beta = -1.2 * sqrt(data.AR) * zw / data.Wingspan * 2 * 1 * 1 * 1 * df / data.Wingspan;

Cl_beta_stable_range = [-0.20, -0.05]; % /rad, typical light GA/RC aircraft -- verify against Sadraey's tables

fprintf('  Dihedral = %.1f deg\n', Gamma_dihedral_deg);
fprintf('  Cl_beta (dihedral only) = %.4f /rad (vs. %.4f /rad at zero dihedral)', Cl_beta_dihedral, Cl_beta);
if Cl_beta_dihedral <= Cl_beta_stable_range(2) && Cl_beta_dihedral >= Cl_beta_stable_range(1)
    fprintf('  [within typical stable range %.2f to %.2f]\n', Cl_beta_stable_range(1), Cl_beta_stable_range(2));
else
    fprintf('  [OUTSIDE typical stable range %.2f to %.2f -- reconcile]\n', Cl_beta_stable_range(1), Cl_beta_stable_range(2));
end

data.StabilityDerivatives.CL_Alpha  = data.CL_Alpha;
data.StabilityDerivatives.x_n       = x_n;
data.StabilityDerivatives.SM        = SM_actual;
data.StabilityDerivatives.CM_Alpha  = CM_Alpha;
data.StabilityDerivatives.Cn_beta   = Cn_beta;
data.StabilityDerivatives.Gamma_deg = Gamma_dihedral_deg;
data.StabilityDerivatives.Cl_beta   = Cl_beta;

end
