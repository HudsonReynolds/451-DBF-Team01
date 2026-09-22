% Main script: Run all of the sizing and analysis in one place. 

clear;clc;close all

Setup() % setup plotting & paths for everything

params = readParams("SizingParams.xlsx");

% run all of the sizing:
InitialVehicleSizingConstraint(params);

% TODO: Weight is both in the excel and here. UPDATE
params.W = VehicleWeightEstimation(params);

% Drag build-up
params.CD_0 = DragBuildUp(params);
% 
% stabilility analysis
A5B(params);

PropulsionSizing(params);