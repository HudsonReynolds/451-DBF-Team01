clear;clc;

Setup() % setup plotting & paths for everything

params = readParams("SizingParams.xlsx");

params = InitialCalcs(params);

params = DragBuildUp(params);

params = calcAero(params);

params = VehicleWeightEstimation(params);

A5B(params);

PropulsionSizing(params);

