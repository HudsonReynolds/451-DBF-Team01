% this script runs all of the scripts for the aircraft sizing:
clear;clc;close all

Setup() % setup plotting & paths for everything

% load the values from the excel sheet
params = readcell("SizingParams.xlsx");

N = length(params);

% put the data into a MATLAB 
for idx = 2:N
    data.(params{idx,1}) = params{idx,2};
end

% run all of the sizing:
InitialVehicleSizingConstraint(data);
 
data.W = VehicleWeightEstimation(data);

% Drag build-up
data.CD_0 = DragBuildUp(data);

% stabilility analysis
A5B(data);