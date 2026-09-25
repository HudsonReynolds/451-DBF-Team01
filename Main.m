clear;clc;

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

A5B(params);

PropulsionSizing(params);

