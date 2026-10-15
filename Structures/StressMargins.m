function params = StressMargins(params)

%% Loads
ult_factor = 1.5;
M_wing_ult = abs(params.structures.wing_M_driving) * ult_factor;
V_wing_ult = abs(params.structures.wing_V_driving) * ult_factor;
T_wing_ult = abs(params.structures.wing_T_driving) * ult_factor;

T_tail_boom_ult = abs(params.structures.tailcone_T) * ult_factor;
V_tail_boom_ult = abs(params.structures.tailcone_V) * ult_factor;
M_tail_boom_ult = abs(params.structures.tailcone_Ml) * ult_factor;

M_fuselage_ult = abs(params.structures.fuselage_M_envelope_max) * ult_factor;
V_fuselage_ult = abs(params.structures.fuselage_V_envelope_max) * ult_factor;
T_fuselage_ult = abs(params.structures.tailcone_T) * ult_factor;

F_landing_ult = abs(params.structures.landing_F_main) * ult_factor;

%% Wing spar
% Dimensions
width_spar_out = params.geometry.width_spar;
height_spar_out = params.geometry.width_spar;
wall_thick_spar = params.geometry.wall_thick_spar;

% Material properties (AL 6061-T6) 
spar_fty = params.materials.fty_al_6061_t6 * 10^6;
spar_fsu = params.materials.fsu_al_6061_t6 * 10^6;  

% Inner dimensions and moment of inertia
width_spar_in = width_spar_out - 2*wall_thick_spar;
height_spar_in = height_spar_out - 2*wall_thick_spar;
I_spar = (width_spar_out * height_spar_out^3)/12 - (width_spar_in * height_spar_in^3)/12;

% Bending stress
y_max_spar = height_spar_out / 2;
sigma_bend_spar = (M_wing_ult * y_max_spar) / I_spar;

% Transverse shear stress
Q_spar = (width_spar_out * height_spar_out^2)/8 - (width_spar_in * height_spar_in^2)/8;
tau_shear_spar = (V_wing_ult * Q_spar) / (I_spar * 2 * wall_thick_spar);

% Torsional shear stress (thin walled tube)
A_m_spar = (width_spar_out - wall_thick_spar) * (height_spar_out - wall_thick_spar); 
tau_torsion_spar = T_wing_ult / (2 * A_m_spar * wall_thick_spar);

% Von-Mises stress
sigma_vm_spar = sqrt(sigma_bend_spar^2 + 3 * tau_torsion_spar^2);

% Max shear stress on one side wall
tau_spar_max = tau_shear_spar + tau_torsion_spar;

% Store stresses
params.structures.sigma_bend_spar = sigma_bend_spar / 1e6;
params.structures.tau_shear_spar = tau_shear_spar / 1e6;
params.structures.tau_torsion_spar = tau_torsion_spar / 1e6;
params.structures.sigma_vm_spar = sigma_vm_spar / 1e6;
params.structures.tau_spar_max = tau_spar_max / 1e6;

% Wing spar margins
params.structures.MoS_spar_vm = (spar_fty / sigma_vm_spar) - 1;
params.structures.MoS_spar_shear = (spar_fsu / tau_spar_max) - 1;

%% Tail boom
% Dimensions
width_tail_boom_out = params.geometry.width_tail_boom;
height_tail_boom_out = params.geometry.width_tail_boom;
wall_thick_tail_boom = params.geometry.wall_thick_tail_boom;
length_tail_boom = params.geometry.length_tail_boom;

% Material properties (AL 6061-T6)
tail_boom_fty = params.materials.fty_al_6061_t6 * 10^6;
tail_boom_fsu = params.materials.fsu_al_6061_t6 * 10^6;
E_tail_boom = params.materials.E_al_6061_t6 * 10^6;

% Inner dimensions and moment of inertia
width_tail_boom_in = width_tail_boom_out - 2*wall_thick_tail_boom;
height_tail_boom_in = height_tail_boom_out - 2*wall_thick_tail_boom;
I_tail_boom = (width_tail_boom_out * height_tail_boom_out^3)/12 - (width_tail_boom_in * height_tail_boom_in^3)/12;

% Tail deflection
V_tail_boom = V_tail_boom_ult / ult_factor;
params.structures.tail_def_angle = rad2deg((V_tail_boom * length_tail_boom^2) / (2 * E_tail_boom * I_tail_boom));
params.structures.tail_def = (V_tail_boom * length_tail_boom^3) / (3 * E_tail_boom * I_tail_boom);

% Bending stress
y_max_tail_boom = height_tail_boom_out / 2;
sigma_bend_tail_boom = (M_tail_boom_ult * y_max_tail_boom) / I_tail_boom;

% Transverse shear stress
Q_tail_boom = (width_tail_boom_out * height_tail_boom_out^2)/8 - (width_tail_boom_in * height_tail_boom_in^2)/8;
tau_shear_tail_boom = (V_tail_boom_ult * Q_tail_boom) / (I_tail_boom * 2 * wall_thick_tail_boom);

% Torsional shear stress (thin walled tube)
A_m_tail_boom = (width_tail_boom_out - wall_thick_tail_boom) * (height_tail_boom_out - wall_thick_tail_boom);
tau_torsion_tail_boom = T_tail_boom_ult / (2 * A_m_tail_boom * wall_thick_tail_boom);

% Von-Mises stress
sigma_vm_tail_boom = sqrt(sigma_bend_tail_boom^2 + 3 * tau_torsion_tail_boom^2);

% Max shear stress on one side wall
tau_tail_boom_max = tau_shear_tail_boom + tau_torsion_tail_boom;

% Store stresses
params.structures.sigma_bend_tail_boom = sigma_bend_tail_boom / 1e6;
params.structures.tau_shear_tail_boom = tau_shear_tail_boom / 1e6;
params.structures.tau_torsion_tail_boom = tau_torsion_tail_boom / 1e6;
params.structures.sigma_vm_tail_boom = sigma_vm_tail_boom / 1e6;
params.structures.tau_tail_boom_max = tau_tail_boom_max / 1e6;

% Tail boom margins
params.structures.MoS_tail_boom_vm = (tail_boom_fty / sigma_vm_tail_boom) - 1;
params.structures.MoS_tail_boom_shear = (tail_boom_fsu / tau_tail_boom_max) - 1;

%% Fuselage
% Dimensions
width_fuselage_out = params.geometry.width_fuselage; 
height_fuselage_out = params.geometry.height_fuselage; 
wall_thick_fuselage = params.geometry.wall_thick_fuselage;

% Material properties (Plywood)
fuselage_fc = params.materials.fc_plywood * 10^6; 
fuselage_fs = params.materials.fs_panel_plywood * 10^6; 

% Inner dimensions and moment of inertia
width_fuselage_in = width_fuselage_out - 2*wall_thick_fuselage;
height_fuselage_in = height_fuselage_out - 2*wall_thick_fuselage;
I_fuselage = (width_fuselage_out * height_fuselage_out^3)/12 - (width_fuselage_in * height_fuselage_in^3)/12;

% Bending stress
y_max_fuselage = height_fuselage_out / 2;
sigma_bend_fuselage = (M_fuselage_ult * y_max_fuselage) / I_fuselage;

% Transverse shear stress
Q_fuselage = (width_fuselage_out * height_fuselage_out^2)/8 - (width_fuselage_in * height_fuselage_in^2)/8;
tau_shear_fuselage = (V_fuselage_ult * Q_fuselage) / (I_fuselage * 2 * wall_thick_fuselage);

% Torsional shear stress (thin walled tube)
A_m_fuselage = (width_fuselage_out - wall_thick_fuselage) * (height_fuselage_out - wall_thick_fuselage);
tau_torsion_fuselage = T_fuselage_ult / (2 * A_m_fuselage * wall_thick_fuselage);

% Max shear stress on one side wall
tau_combined_fuselage = tau_torsion_fuselage + tau_shear_fuselage;

% Store stresses [MPa]
params.structures.sigma_bend_fuselage = sigma_bend_fuselage / 1e6;
params.structures.tau_shear_fuselage = tau_shear_fuselage / 1e6;
params.structures.tau_torsion_fuselage = tau_torsion_fuselage / 1e6;
params.structures.tau_combined_fuselage = tau_combined_fuselage / 1e6;

% Fuselage margins
params.structures.MoS_fuselage_bend = (fuselage_fc / sigma_bend_fuselage) - 1;
params.structures.MoS_fuselage_shear = (fuselage_fs / tau_combined_fuselage) - 1;

%% Landing gear
% Dimensions
gear_strut_length = params.geometry.gear_strut_length;
gear_strut_width_max = params.geometry.gear_strut_width_max;
gear_strut_width_min = params.geometry.gear_strut_width_min;
gear_strut_thickness = params.geometry.gear_strut_thickness;
gear_strut_angle = params.geometry.gear_strut_angle;

% Material properties
gear_fty = params.materials.fty_al_6061_t6 * 10^6;
gear_fsu = params.materials.fsu_al_6061_t6 * 10^6;

% Loads
P_axial = F_landing_ult * sind(gear_strut_angle) / 2;
P_trans = F_landing_ult * cosd(gear_strut_angle) / 2;

% Bending stress
M_gear = P_trans * gear_strut_length;
I_gear = (gear_strut_width_max * gear_strut_thickness^3) / 12;
y_max_gear = gear_strut_thickness / 2;
sigma_bend_gear = (M_gear * y_max_gear) / I_gear;

% Compressive stress
A_strut_min = gear_strut_width_min * gear_strut_thickness;
sigma_comp_gear = P_axial / A_strut_min;

% Max shear stress
tau_gear = 1.5 * P_trans / A_strut_min;

% Store stresses
params.structures.sigma_bend_gear = sigma_bend_gear / 1e6;
params.structures.sigma_comp_gear = sigma_comp_gear / 1e6;
params.structures.tau_gear = tau_gear / 1e6;

% Gear margins
sigma_vm_gear_outer = sigma_bend_gear + sigma_comp_gear;              
sigma_vm_gear_NA = sqrt(sigma_comp_gear^2 + 3 * tau_gear^2);          
sigma_vm_gear = max(sigma_vm_gear_outer, sigma_vm_gear_NA);

params.structures.MoS_gear_vm = (gear_fty / sigma_vm_gear) - 1;
params.structures.MoS_gear_shear = (gear_fsu / tau_gear) - 1;