% A8 Deliverable 7
function params = Joints(params)

%% Dimensions
% Bolts
bolt_OD = params.geometry.bolt_OD;

% Washers
washer_OD = params.geometry.washer_OD;
washer_ID = params.geometry.washer_ID;
washer_area = 0.25 * pi * (washer_OD^2 - washer_ID^2);

% Aluminum tubes
wall_thick_tube = params.geometry.wall_thick_spar;
width_tail_boom = params.geometry.width_tail_boom;

% Plywood
wall_thick_fuselage = params.geometry.wall_thick_fuselage;

%% Material properties
% Plywood
fs_panel_plywood = params.materials.fs_panel_plywood * 10^6; % For shear perpendicular to plys
fs_planar_plywood = params.materials.fs_planar_plywood * 10^6; % For shear parallel to plys (weaker)

% Aluminum tube
fsu_al_6061_t6 = params.materials.fsu_al_6061_t6 * 10^6;
fbry_al_6061_t6 = params.materials.fbry_al_6061_t6 * 10^6;

%% Wing to fuselage
% Load divided among all 6 bolts
% Margins evaluated for bolting into plywood and aluminum, min taken

% Loads
ult_factor = 1.5;
V_wing_ult = 2 * abs(params.structures.wing_V_driving) * ult_factor;
no_bolts_al = 2;
no_bolts_plywood = 4;
load_per_bolt = V_wing_ult / (no_bolts_al + no_bolts_plywood);

% Stress
shear_area = pi * washer_OD * wall_thick_fuselage;
tau = load_per_bolt / shear_area;

% Margins
params.structures.MoS_wing_joint = (fs_panel_plywood / tau) - 1;

%% Tail boom joint
% Only evaluating bolts at boom split (sees worst loads)
% Assumes one bolt carries entire load

% Loads
ult_factor = 1.5;
V_tail_boom_ult = abs(params.structures.tailcone_V) * ult_factor;
T_tail_boom_ult = abs(params.structures.tailcone_T) * ult_factor;
P_boom = V_tail_boom_ult + T_tail_boom_ult / width_tail_boom;

% Stress
bearing_area = 2 * wall_thick_tube * bolt_OD;
sigma_br = P_boom / bearing_area;

% MargiV
params.structures.MoS_boom_joint = (fbry_al_6061_t6 / sigma_br) - 1;

%% Motor to firewall

% Loads
T_static_ult = params.prop.T_static_actual * ult_factor;

% Stress
no_bolts = 4;
shear_area = no_bolts * pi * bolt_OD * wall_thick_fuselage;
tau = T_static_ult / shear_area;

% Margins
params.structures.MoS_motor_mount = (fs_panel_plywood / tau) - 1;

%% Landing gear

% Loads
F_landing_ult = abs(params.structures.landing_F_main) * ult_factor;
mu_L = params.env.mu_L;
V_landing = mu_L * F_landing_ult;

% Stress
no_bolts = 4;
bearing_area = no_bolts * wall_thick_fuselage * bolt_OD;
sigma_br = V_landing / bearing_area;

% Margins
params.structures.MoS_gear_mount = (fs_planar_plywood / tau) - 1; % Probaly wrong strength to compare, idk what bearing strength is

end