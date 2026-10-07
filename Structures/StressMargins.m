% A8 Deliverables 5 and 6
function params = StressMargins(params)

%% Loads
ult_factor = 1.5;
M_wing_ult = params.structures.wing_M_driving * ult_factor;
T_wing_ult = params.structures.wing_T_driving * ult_factor;

T_tail_boom_ult = params.structures.tailcone_T * ult_factor;
M_tail_boom_ult = params.structures.tailcone_Ml * ult_factor;

M_fuselage_ult  = params.structures.fuselage_M_envelope_max * ult_factor;
V_fuselage_ult  = params.structures.fuselage_V_envelope_max * ult_factor;
T_fuselage_ult  = params.structures.tailcone_T * ult_factor; 

F_landing_ult = (params.structures.landing_F_main) * ult_factor;

%% Wing spar
% Dimensions
width_spar_out  = params.geometry.width_spar;
height_spar_out = params.geometry.width_spar;
length_spar = params.geometry.length_spar;
wall_thick_spar = params.geometry.wall_thick_spar;

% Material properties (AL 6061-T6)
spar_E = params.materials.E_al_6061_t6 * 10^6; 
spar_fty = params.materials.fty_al_6061_t6 * 10^6;
spar_fsu = params.materials.fsu_al_6061_t6 * 10^6;  

% Inner dimensions and moment of inertia
width_spar_in = width_spar_out - 2*wall_thick_spar;
height_spar_in = height_spar_out - 2*wall_thick_spar;
I_spar = (width_spar_out * height_spar_out^3)/12 - (width_spar_in * height_spar_in^3)/12;

% Bending stress
y_max_spar = height_spar_out / 2;
sigma_bend_spar = (M_wing_ult * y_max_spar) / I_spar;

% Torsional shear stress (thin walled tube)
A_m_spar = (width_spar_out - wall_thick_spar) * (height_spar_out - wall_thick_spar); 
tau_torsion_spar = T_wing_ult / (2 * A_m_spar * wall_thick_spar);

% Von-Mises stress
sigma_vm_spar = sqrt(sigma_bend_spar^2 + 3 * tau_torsion_spar^2);

% Wing tip deflection
deflection_tip = (params.structures.wing_M_driving * length_spar^2) / (4 * spar_E * I_spar);

% Wing spar margins
params.structures.MoS_spar_vm = (spar_fty / sigma_vm_spar) - 1;

%% Tail boom
% Dimensions
width_tail_boom_out  = params.geometry.width_tail_boom;
height_tail_boom_out = params.geometry.width_tail_boom;
length_tail_boom = params.geometry.length_tail_boom; % [m] Exposed length outside the fuselage
wall_thick_tail_boom = params.geometry.wall_thick_tail_boom;

% Material properties (AL 6061-T6)
tail_boom_E = params.materials.E_al_6061_t6 * 10^6;
tail_boom_fty = params.materials.fty_al_6061_t6 * 10^6;
tail_boom_fsu = params.materials.fsu_al_6061_t6 * 10^6;

% Inner dimensions and moment of inertia
width_tail_boom_in = width_tail_boom_out - 2*wall_thick_tail_boom;
height_tail_boom_in = height_tail_boom_out - 2*wall_thick_tail_boom;
I_tail_boom = (width_tail_boom_out * height_tail_boom_out^3)/12 - (width_tail_boom_in * height_tail_boom_in^3)/12;

% Bending stress
y_max_tail_boom = height_tail_boom_out / 2;
sigma_bend_tail_boom = (M_tail_boom_ult * y_max_tail_boom) / I_tail_boom;

% Torsional shear stress (thin walled tube)
A_m_tail_boom = (width_tail_boom_out - wall_thick_tail_boom) * (height_tail_boom_out - wall_thick_tail_boom);
tau_torsion_tail_boom = T_tail_boom_ult / (2 * A_m_tail_boom * wall_thick_tail_boom);

% Von-Mises stress
sigma_vm_tail_boom = sqrt(sigma_bend_tail_boom^2 + 3 * tau_torsion_tail_boom^2);

% Tail boom tip deflection (cantilever, tip load: delta = P*L^3/(3EI) = M_root*L^2/(3EI))
deflection_tip_tail_boom = (params.structures.tailcone_Ml * length_tail_boom^2) / (3 * tail_boom_E * I_tail_boom);

% Tail boom margins
params.structures.MoS_tail_boom_vm = (tail_boom_fty / sigma_vm_tail_boom) - 1;

%% Fuselage
% Dimensions
width_fuselage_out = params.geometry.width_fuselage; 
height_fuselage_out = params.geometry.height_fuselage; 
wall_thick_fuselage = params.geometry.wall_thick_fuselage;

% Material properties (Plywood)
fuselage_fa = params.materials.fa_plywood * 10^6; % Plywood allowable stress

% Inner dimensions and moment of inertia
width_fuselage_in = width_fuselage_out - 2*wall_thick_fuselage;
height_fuselage_in = height_fuselage_out - 2*wall_thick_fuselage;
I_fuselage = (width_fuselage_out * height_fuselage_out^3)/12 - (width_fuselage_in * height_fuselage_in^3)/12;

% Bending stress
y_max_fuselage = height_fuselage_out / 2;
sigma_bend_fuselage = (M_fuselage_ult * y_max_fuselage) / I_fuselage;

% Torsional shear stress (thin walled tube)
A_m_fuselage = (width_fuselage_out - wall_thick_fuselage) * (height_fuselage_out - wall_thick_fuselage);
tau_torsion_fuselage = T_fuselage_ult / (2 * A_m_fuselage * wall_thick_fuselage);

% Vertical shear stress (carried by the two side walls)
height_fuselage_mid = height_fuselage_out - wall_thick_fuselage;
tau_shear_fuselage = V_fuselage_ult / (2 * height_fuselage_mid * wall_thick_fuselage);

% Combined shear on the critical side wall (torsion and vertical shear add on one side)
tau_combined_fuselage = tau_torsion_fuselage + tau_shear_fuselage;

% Fuselage margins
params.structures.MoS_fuselage_bend = (fuselage_fa / sigma_bend_fuselage) - 1;
params.structures.MoS_fuselage_shear = (fuselage_fa / tau_combined_fuselage) - 1;

%% Landing gear
% Dimensions
gear_strut_length = params.geometry.gear_strut_length;
gear_strut_width_max = params.geometry.gear_strut_width_max;
gear_strut_width_min = params.geometry.gear_strut_width_min;
gear_strut_thickness = params.geometry.gear_strut_thickness;
gear_strut_angle = params.geometry.gear_strut_angle;

% Material properties
gear_fty = params.materials.fty_al_6061_t6 * 10^6;

% Loads
P_bending = F_landing_ult * cosd(gear_strut_angle) / 2;
P_comp = F_landing_ult * sind(gear_strut_angle) / 2;

% Bending stress
M_gear = P_bending * gear_strut_length;
I_gear = (gear_strut_width_max * gear_strut_thickness^3) / 12;
y_max_gear = gear_strut_thickness / 2;
sigma_bend_gear = (M_gear * y_max_gear) / I_gear;

% Compressive stress
A_strut_min = gear_strut_width_min * gear_strut_thickness;
sigma_comp_gear = P_comp / A_strut_min;

% Shear stress


% Von-Mises stress
sigma_total = sigma_bend_gear + sigma_comp_gear;

% Gear margins
params.structures.MoS_gear_vm = (gear_fty / sigma_vm_gear) - 1;
