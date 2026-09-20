function outputs = StabilityDerivatives(params)

SM_actual = params.geometry.x_n - params.geometry.x_cg_design;
CM_Alpha  = -params.aero.CL_Alpha*SM_actual;

fprintf('--- Longitudinal derivatives (CG = %.2f MAC) ---\n', params.geometry.x_cg_design);
fprintf('  CL_alpha (aircraft, 3-D)  = %.3f /rad\n', params.aero.CL_Alpha);
fprintf('  Neutral point x_n         = %.3f MAC\n', params.geometry.x_n);
fprintf('  Static margin (from x_n)  = %.3f  (design assumption was %.2f -- reconcile)\n', SM_actual, params.geometry.SM);
fprintf('  CM_alpha                  = %.3f /rad\n', CM_Alpha);
if CM_Alpha > 0
    warning('CM_alpha is positive -- CG is aft of the neutral point, statically unstable. Fix the scissor plot inputs before trusting anything downstream.');
end

%% ---- Lateral-directional: Cn_beta, Cl_beta ----

Cn_beta_stable_range = [0.05, 0.15]; % /rad, typical light GA/RC aircraft

fprintf('--- Lateral-directional derivatives ---\n');
fprintf('  Cn_beta (VT only) = %.4f /rad', params.aero.Cn_beta_VT);
if params.aero.Cn_beta_VT >= Cn_beta_stable_range(1) && params.aero.Cn_beta_VT <= Cn_beta_stable_range(2)
    fprintf('  [within typical stable range %.2f-%.2f]\n', Cn_beta_stable_range(1), Cn_beta_stable_range(2));
else
    fprintf('  [OUTSIDE typical stable range %.2f-%.2f -- reconcile]\n', Cn_beta_stable_range(1), Cn_beta_stable_range(2));
end

% Dihedral effect on Cl_beta, via the same strip-theory approach used for
% roll damping/roll control power in ControlSurfaceSizing.m -- shown as an
% integral (not a memorized coefficient) so it's auditable:
Gamma_dihedral = deg2rad(params.geometry.Gamma_dihedral_deg);

Cl_beta_dihedral = -0.5 * Gamma_dihedral * params.aero.CL_Alpha_Wing;

Cl_beta = (-1.2 * sqrt(params.geometry.AR) * params.geometry.zw / params.geometry.wingspan * 2 * params.geometry.df / params.geometry.wingspan) + Cl_beta_dihedral;

Cl_beta_stable_range = [-0.20, -0.05]; % /rad, typical light GA/RC aircraft

fprintf('  Dihedral = %.1f deg\n', params.geometry.Gamma_dihedral_deg);
fprintf('  Cl_beta (dihedral only) = %.4f /rad (vs. %.4f /rad at zero dihedral)', Cl_beta_dihedral, Cl_beta);
if Cl_beta_dihedral <= Cl_beta_stable_range(2) && Cl_beta_dihedral >= Cl_beta_stable_range(1)
    fprintf('  [within typical stable range %.2f to %.2f]\n', Cl_beta_stable_range(1), Cl_beta_stable_range(2));
else
    fprintf('  [OUTSIDE typical stable range %.2f to %.2f -- reconcile]\n', Cl_beta_stable_range(1), Cl_beta_stable_range(2));
end

outputs.StabilityDerivatives.CL_Alpha  = params.aero.CL_Alpha;
outputs.StabilityDerivatives.x_n       = params.geometry.x_n;
outputs.StabilityDerivatives.SM        = SM_actual;
outputs.StabilityDerivatives.CM_Alpha  = CM_Alpha;
outputs.StabilityDerivatives.Cn_beta   = params.aero.Cn_beta_VT;
outputs.StabilityDerivatives.Gamma_deg = params.geometry.Gamma_dihedral_deg;
outputs.StabilityDerivatives.Cl_beta   = Cl_beta;

end
