clear;clc;close all;

Setup() % setup plotting & paths for everything

params = readParams("SizingParams.xlsx");

% Iterate between vehicle weight and aerodynamic performance
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

figs_before_preview = findall(0, 'Type', 'figure');
evalc('[~, params] = PropulsionDragModel(params); [~, params] = CruisePerformance(params);');
delete(setdiff(findall(0, 'Type', 'figure'), figs_before_preview));

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

fprintf('\n========================================\n');
fprintf('  AIRCRAFT PERFORMANCE (A9, Deliverables 1-5)\n');
fprintf('========================================\n');

[~, params] = PropulsionDragModel(params);

[~, params] = TakeoffPerformance(params);

[~, params] = ClimbPerformance(params);

[~, params] = CruisePerformance(params);

[~, params] = TurnPerformance(params);

fprintf('\n========================================\n');
fprintf('  AIRCRAFT PERFORMANCE (A9, Deliverables 6-9)\n');
fprintf('========================================\n');

[~, params] = MissionSimulation(params);

[~, params] = AssumptionValidation(params);

[~, params] = MaxPayloadSweep(params);

[~, params] = RequirementsValidation(params);
 

% Save every figure generated above into Plots/, replacing whatever was
% there before -- so Plots/ always matches exactly this run's figures.
% ExportAllFigures('Plots');

writeOutputs(params, "SizingParams.xlsx");

% Save every figure generated above into Plots/, replacing whatever was
% there before -- so Plots/ always matches exactly this run's figures.
ExportAllFigures('Plots');

