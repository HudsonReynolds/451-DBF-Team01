% A8 Deliverables 5 and 6
function [outputs, params] = StressMargins(params)

%% ========================================================================
%  AAE 451: DELIVERABLES 5 & 6 
%  Deliverable 5: Stress Calculations, Component Sizing, & Shear Flow Diagram
%  Deliverable 6: Margins of Safety (MoS) Table
%  ========================================================================

fprintf('\n========================================\n');
fprintf('  DELIVERABLE 5 & 6: STRESS & MARGINS\n');
fprintf('========================================\n');

% [DELIVERABLE 6]: "Quote every margin at ultimate."
% Extract maximum limit loads from previous scripts and apply 1.5x Ultimate Factor
ult_factor = 1.5;
M_wing_ult = params.structures.wing_M_driving * ult_factor;
T_wing_ult = params.structures.wing_T_driving * ult_factor;

M_fus_ult  = params.structures.fuselage_M_envelope_max * ult_factor;
V_fus_ult  = params.structures.fuselage_V_envelope_max * ult_factor;
T_fus_ult  = params.structures.tailcone_T * ult_factor; 

%% ------------------------------------------------------------------------
%  1. WING SPAR TRADE STUDY (Aluminum 6061-T6 Square Tube)
%  ------------------------------------------------------------------------

% Dimensions from CAD (converted from inches/thou to meters)
IN2M = 0.0254;
spar_width_out  = 0.75 * IN2M;  
spar_height_out = 0.75 * IN2M;  
spar_wall_thick = 0.060 * IN2M; 
spar_span       = params.geometry.wingspan / 2; 

% Aluminum 6061-T6 Material Properties
spar_E        = 69.0e9; 
spar_yield    = 276e6;  % [DELIVERABLE 5]: Sizing against Yield Strength
spar_shear_al = 207e6;  

% [DELIVERABLE 5]: "Spar trade study: bending stress, torsional shear stress... and deflection"
% Inner dimensions and moment of inertia
spar_width_in  = spar_width_out - 2*spar_wall_thick;
spar_height_in = spar_height_out - 2*spar_wall_thick;
I_xx_spar = (spar_width_out * spar_height_out^3)/12 - (spar_width_in * spar_height_in^3)/12;

% Bending Stress (sigma = M*y / I)
y_max_spar = spar_height_out / 2;
sigma_bend_spar = (M_wing_ult * y_max_spar) / I_xx_spar;

% Torsional Shear Stress (Thin-walled tube: tau = T / 2*A_e*t)
A_e_spar = (spar_width_out - spar_wall_thick) * (spar_height_out - spar_wall_thick); 
tau_torsion_spar = T_wing_ult / (2 * A_e_spar * spar_wall_thick);

% [DELIVERABLE 5]: "State the rule you used to combine stresses." (Von Mises)
sigma_von_mises = sqrt(sigma_bend_spar^2 + 3 * tau_torsion_spar^2);

% [DELIVERABLE 5]: "Report deflection wherever it is a design limit."
deflection_tip = (params.structures.wing_M_driving * spar_span^2) / (4 * spar_E * I_xx_spar);

% [DELIVERABLE 6]: "Create a summary table... failure mode and margin." 
% (These calculate the values you will put into your slide deck table)
MoS_spar_bend = (spar_yield / sigma_bend_spar) - 1;
MoS_spar_shear = (spar_shear_al / tau_torsion_spar) - 1;
MoS_spar_von_mises = (spar_yield / sigma_von_mises) - 1;

fprintf('\n--- Wing Spar (Al 6061 Square Tube) ---\n');
fprintf('  Dimensions:              0.75" x 0.75" x 0.060" wall\n');
fprintf('  Ultimate Bending Moment: %.2f N*m\n', M_wing_ult);
fprintf('  [D5] Bending Stress:     %.2f MPa\n', sigma_bend_spar / 1e6);
fprintf('  [D6] Bending MoS (Yield):%+.2f\n', MoS_spar_bend);
fprintf('  [D5] Torsional Shear:    %.2f MPa\n', tau_torsion_spar / 1e6);
fprintf('  [D6] Torsion Shear MoS:  %+.2f\n', MoS_spar_shear);
fprintf('  [D5] Combined Rule:      Von Mises Criterion\n');
fprintf('  [D5] Von Mises Stress:   %.2f MPa\n', sigma_von_mises / 1e6);
fprintf('  [D6] Von Mises MoS(Yield):%+.2f\n', MoS_spar_von_mises);
fprintf('  [D5] Tip Deflection:     %.1f mm\n', deflection_tip * 1000);


%% ------------------------------------------------------------------------
%  2. FUSELAGE BOOM IDEALIZATION (4-Boom Rectangular Plywood Box)
%  ------------------------------------------------------------------------

% >>> TODO: REPLACE THESE PLACEHOLDERS WITH REAL CAD DATA <<<
fus_w  = 0.100; % [m] Outer width of fuselage
fus_h  = 0.120; % [m] Outer height of fuselage
ply_t  = 0.003; % [m] Plywood wall/skin thickness 
frame_pitch = 0.150; % [m] Distance between internal frames

% Plywood Material Properties (Placeholder standard values)
ply_E        = 9.0e9; 
ply_yield    = 30e6;  % [DELIVERABLE 5]: Sizing against Yield Strength
ply_shear_al = 6.0e6; 

% [DELIVERABLE 5]: "...present the calculation: the boom areas and positions"
% Area of corners (assuming folded ply corners, effective area approx = t^2)
boom_area = ply_t * ply_t; 
B_i = [boom_area, boom_area, boom_area, boom_area];

% Boom coordinates from centroid (1=TopRight, 2=BotRight, 3=BotLeft, 4=TopLeft)
z_b = [fus_h/2, -fus_h/2, -fus_h/2, fus_h/2];
y_b = [fus_w/2,  fus_w/2, -fus_w/2, -fus_w/2];

% [DELIVERABLE 5]: "...the second moment of area, the direct stress in each boom"
I_yy_fus = sum(B_i .* z_b.^2);
sigma_booms = (M_fus_ult .* z_b) ./ I_yy_fus;
[max_comp_stress, idx_crit_boom] = max(abs(sigma_booms));

% [DELIVERABLE 5]: "Size every member... for buckling."
% Euler Column Buckling for the Plywood Corners
corner_I = (ply_t * ply_t^3) / 12; 
sigma_cr_corner = (pi^2 * ply_E * corner_I) / (boom_area * frame_pitch^2);

% [DELIVERABLE 5]: "...the open-section, closing and torsion shear flows"
% 1. Open Section (q_b): Walk clockwise. Cut the Top panel (Panel 4->1, qb = 0)
q_b = zeros(1, 4);
q_b(1) = 0 - (V_fus_ult / I_yy_fus) * (B_i(1) * z_b(1));
q_b(2) = q_b(1) - (V_fus_ult / I_yy_fus) * (B_i(2) * z_b(2));
q_b(3) = q_b(2) - (V_fus_ult / I_yy_fus) * (B_i(3) * z_b(3));
q_b(4) = 0; 

% 2. Closing Flow (q_s0) and Torsion (q_T)
A_cell = fus_w * fus_h; 
M_qb = q_b(1)*fus_h*(fus_w/2) + q_b(2)*fus_w*(fus_h/2) + q_b(3)*fus_h*(fus_w/2) + q_b(4)*fus_w*(fus_h/2);
q_s0 = -M_qb / (2 * A_cell); 
q_T = T_fus_ult / (2 * A_cell); 

% [DELIVERABLE 5]: "...the total shear flow and shear stress in that panel"
q_total = q_b + q_s0 + q_T;
tau_panels = abs(q_total) / ply_t;
[max_tau, idx_crit_panel] = max(tau_panels);

% [DELIVERABLE 6]: "Every margin of safety must be zero or greater"
MoS_corner_yield = (ply_yield / max_comp_stress) - 1;
MoS_corner_buckling = (sigma_cr_corner / max_comp_stress) - 1;
MoS_skin_shear = (ply_shear_al / max_tau) - 1; 

fprintf('\n--- Fuselage Box (4-Boom Idealization) ---\n');
fprintf('  Ultimate Bending Moment: %.2f N*m\n', M_fus_ult);
fprintf('  [D5] Max Corner Stress:  %.2f MPa (Boom %d)\n', max_comp_stress / 1e6, idx_crit_boom);
fprintf('  [D6] Corner MoS (Yield): %+.2f\n', MoS_corner_yield);
fprintf('  [D5] Euler Buckling Limit:%.2f MPa\n', sigma_cr_corner / 1e6);
fprintf('  [D6] Buckling MoS:       %+.2f\n\n', MoS_corner_buckling);

fprintf('  [D5] Ultimate Shear (V): %.2f N\n', V_fus_ult);
fprintf('  [D5] Closing Flow (q_s0):%.3f N/mm\n', q_s0 / 1000);
fprintf('  [D5] Torsion Flow (q_T): %.3f N/mm\n', q_T / 1000);
% [DELIVERABLE 5]: "...and a shear-flow diagram of the section." 
fprintf('  [D5] Max Panel Flow:     %.3f N/mm (Panel %d) <-- USE FOR DIAGRAM\n', max(abs(q_total)) / 1000, idx_crit_panel);
fprintf('  [D5] Max Shear Stress:   %.2f MPa\n', max_tau / 1e6);
fprintf('  [D6] Skin Shear MoS:     %+.2f\n', MoS_skin_shear);


%% ------------------------------------------------------------------------
%  3. FUSELAGE BULKHEAD / FRAME (Wing Attachment Point)
%  ------------------------------------------------------------------------

P_wing = params.structures.Lw_maneuver * ult_factor; % Ultimate wing reaction force

% >>> TODO: REPLACE FRAME CROSS SECTION <<<
frame_width  = 0.005; % [m] Width of the plywood bulkhead
frame_height = 0.012; % [m] Depth of the bulkhead ring
fus_radius   = fus_w / 2; % Approx radius for ring frame eq

% [DELIVERABLE 5]: "Size every member in your load path"
M_frame_max = 0.239 * P_wing * fus_radius; % Bruhn/Megson ring frame equation
Z_frame = (frame_width * frame_height^2) / 6; % Section modulus
sigma_frame = M_frame_max / Z_frame;

% [DELIVERABLE 6]: "...summary table for all components"
MoS_frame = (ply_yield / sigma_frame) - 1;

fprintf('\n--- Fuselage Frame / Bulkhead ---\n');
fprintf('  Wing Reaction Load:      %.2f N\n', P_wing);
fprintf('  [D5] Max Frame Bending:  %.2f N*m\n', M_frame_max);
fprintf('  [D5] Frame Stress:       %.2f MPa\n', sigma_frame / 1e6);
fprintf('  [D6] Frame MoS (Yield):  %+.2f\n', MoS_frame);


%% ------------------------------------------------------------------------
%  4. TAIL BOOM (Aluminum 6061-T6 Square Tube)
%  ------------------------------------------------------------------------

% >>> TODO: REPLACE EXPOSED LENGTH AND TAIL LOAD <<<
tail_boom_length = 0.800; % [m] Exposed length outside the fuselage
P_tail_ult = 50.0 * ult_factor; % [N] Ultimate aerodynamic force on the tail

% [DELIVERABLE 5]: "Size every member in your load path"
% Shares I_xx_spar, A_e_spar, and spar_yield from the Wing calculation
M_boom_ult = P_tail_ult * tail_boom_length; 

% Bending Stress & Torsional Shear
sigma_bend_boom = (M_boom_ult * y_max_spar) / I_xx_spar;
tau_torsion_boom = T_fus_ult / (2 * A_e_spar * spar_wall_thick);

% [DELIVERABLE 5]: "State the rule you used to combine stresses." (Von Mises)
sigma_von_mises_boom = sqrt(sigma_bend_boom^2 + 3 * tau_torsion_boom^2);

% [DELIVERABLE 6]: "...summary table for all components"
MoS_boom_bend = (spar_yield / sigma_bend_boom) - 1;
MoS_boom_shear = (spar_shear_al / tau_torsion_boom) - 1;
MoS_boom_von_mises = (spar_yield / sigma_von_mises_boom) - 1;

fprintf('\n--- Tail Boom (Al 6061 Square Tube) ---\n');
fprintf('  Exposed Length:          %.3f m\n', tail_boom_length);
fprintf('  Ultimate Tail Load:      %.2f N\n', P_tail_ult);
fprintf('  [D5] Max Bending Moment: %.2f N*m\n', M_boom_ult);
fprintf('  [D5] Bending Stress:     %.2f MPa\n', sigma_bend_boom / 1e6);
fprintf('  [D6] Bending MoS (Yield):%+.2f\n', MoS_boom_bend);
fprintf('  [D5] Torsional Shear:    %.2f MPa\n', tau_torsion_boom / 1e6);
fprintf('  [D6] Torsion Shear MoS:  %+.2f\n', MoS_boom_shear);
fprintf('  [D5] Von Mises Stress:   %.2f MPa\n', sigma_von_mises_boom / 1e6);
fprintf('  [D6] Von Mises MoS(Yield):%+.2f\n', MoS_boom_von_mises);


%% ------------------------------------------------------------------------
%  5. MAIN LANDING GEAR STRUT (Cantilever Bending at 10g Landing)
%  ------------------------------------------------------------------------

% >>> TODO: REPLACE GEAR GEOMETRY, LOAD, AND MATERIAL PROPERTIES <<<
leg_length = 0.156; % [m] Developed length of the strut from fuselage to wheel
F_v_ult = 97.0 * ult_factor; % [N] Vertical force per wheel during 10g hard landing

% Strut Cross Section (Placeholder: Solid Circular Rod)
strut_diam = 0.00635; % [m]
strut_E = 207e9; % [Pa] Steel music wire modulus
strut_yield = 1650e6; % [Pa] Gear is typically sized to yield so it doesn't bend

%%% --- [DELIVERABLE 5: STRUT STRESS & DEFLECTION] --- %%%
I_strut = (pi * strut_diam^4) / 64;
Z_strut = (pi * strut_diam^3) / 32;

% Stress and Deflection
M_strut_ult = F_v_ult * leg_length; % Simplified worst-case bending at the root
sigma_strut = M_strut_ult / Z_strut;

% [DELIVERABLE 5]: "Report deflection wherever it is a design limit."
delta_strut = (F_v_ult * leg_length^3) / (3 * strut_E * I_strut);

%%% --- [DELIVERABLE 6: STRUT MARGIN OF SAFETY] --- %%%
MoS_strut_yield = (strut_yield / sigma_strut) - 1;

fprintf('\n--- Main Landing Gear Strut ---\n');
fprintf('  Ultimate Wheel Load:     %.2f N\n', F_v_ult);
fprintf('  [D5] Strut Bending Stress:%.2f MPa\n', sigma_strut / 1e6);
fprintf('  [D6] Strut Yield MoS:    %+.2f\n', MoS_strut_yield);
fprintf('  [D5] Strut Deflection:   %.1f mm\n\n', delta_strut * 1000);


%% Output saving
outputs.StressMargins = struct(...
    'MoS_spar_von_mises', MoS_spar_von_mises, ...
    'MoS_corner_yield', MoS_corner_yield, ...
    'MoS_corner_buckling', MoS_corner_buckling, ...
    'MoS_skin_shear', MoS_skin_shear, ...
    'MoS_frame', MoS_frame, ...
    'MoS_boom_von_mises', MoS_boom_von_mises, ...
    'MoS_strut_yield', MoS_strut_yield, ...
    'q_total', q_total); % Array of flows [Right, Bottom, Left, Top] for your shear flow diagram

end