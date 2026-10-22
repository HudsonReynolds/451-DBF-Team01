clear;clc;close all;

Setup() % setup plotting & paths for everything

params = readParams("SizingParams.xlsx");

% Dihedral knockdown: each wing panel's lift tilts by Gamma and the panel
% sees alpha*cos(Gamma), so vertical lift capability scales by cos^2(Gamma)
params.aero.k_dihedral = cosd(params.geometry.Gamma_dihedral_deg)^2; % Dihedral lift knockdown [-]
params.aero.CL_max     = params.aero.k_dihedral * params.aero.CL_max; % Wing CL_max with dihedral knockdown [-]
params.aero.CL_max_neg = params.aero.k_dihedral * params.aero.CL_max_neg; % Negative wing CL_max with dihedral knockdown [-]

MTOM_actual = params.performance.MTOM; % As-built (measured) mass from the Inputs sheet [kg]

% Iterate between vehicle weight and aerodynamic performance -- the wing
% area is hardcoded (Inputs sheet), so this only gives the weight-fraction
% mass/battery estimate; the as-built mass replaces MTOM below
MTOM_guess = params.performance.MTOM;
err = 1;

while err > 0.001
    params = InitialCalcs(params);
    
    params = DragBuildUp(params);
    
    params = calcAero(params);
    
    params = VehicleWeightEstimation(params);

    MTOM_new = params.performance.MTOM;
    err = abs((MTOM_guess - MTOM_new) / MTOM_guess);
    MTOM_guess = MTOM_new;
end

% As-built aircraft: use the measured mass everywhere downstream (sizing,
% stability, propulsion, loads) with the hardcoded wing area
fprintf('Weight-fraction estimate MTOM = %.3f kg; using as-built MTOM = %.3f kg, S = %.4f m^2\n', ...
    params.performance.MTOM, MTOM_actual, params.geometry.S_wing);
params.performance.MTOM = MTOM_actual;
params = InitialCalcs(params);
params = DragBuildUp(params);
params = calcAero(params);

fprintf('\n========================================\n');
fprintf('  Stability (A5B)\n');
fprintf('========================================\n\n');

params = A5B(params);

fprintf('\n========================================\n');
fprintf('  Servo Sizing\n');
fprintf('========================================\n\n');

ServoSizing(params);

params = PropulsionSizing(params);

PropellerCharacterization(params); 

fprintf('\n========================================\n');
fprintf('  STRUCTURES (A8, Deliverables 2-4)\n');
fprintf('========================================\n');

[~, params] = VnDiagram(params);

VnOperatingEnvelope(params);

[~, params] = WingLoads(params);

[~, params] = TailLoads(params);

[~, params] = FuselageLoads(params);

[~, params] = TailDraggerTakeOff(params);

params = StressMargins(params);

params = Joints(params);

% fprintf('\n========================================\n');
% fprintf('  AIRCRAFT PERFORMANCE (A9, Deliverables 1-5)\n');
% fprintf('========================================\n');
% 
% [~, params] = PropulsionDragModel(params);
% 
% [~, params] = TakeoffPerformance(params);
% 
% [~, params] = ClimbPerformance(params);
% 
% [~, params] = CruisePerformance(params);
% 
% [~, params] = TurnPerformance(params);
% S
% fprintf('\n========================================\n');
% fprintf('  AIRCRAFT PERFORMANCE (A9, Deliverables 6-9)\n');
% fprintf('========================================\n');
% 
% [~, params] = MissionSimulation(params);
% 
% [~, params] = AssumptionValidation(params);
% 
% [~, params] = MaxPayloadSweep(params);
% 
% [~, params] = RequirementsValidation(params);
% % 
% % Save every figure generated above into Plots/, replacing whatever was
% % there before -- so Plots/ always matches exactly this run's figures.
% % ExportAllFigures('Plots');
% 
writeOutputs(params, "SizingParams.xlsx");
% 
% % <<<<<<< Updated upstream
% % =======
% % % Save every figure generated above into Plots/, replacing whatever was
% % % there before -- so Plots/ always matches exactly this run's figures.
% % % ExportAllFigures('Plots');
% % >>>>>>> Stashed changes
