% this script runs all of the scripts for the aircraft sizing:
clear;clc;close all

Setup() % setup plotting & paths for everything

params = readParams("SizingParams.xlsx");



% run all of the sizing:
InitialVehicleSizingConstraint(data);
 
data.W = VehicleWeightEstimation(data);

% Drag build-up
data.CD_0 = DragBuildUp(data);

% stabilility analysis
A5B(data);