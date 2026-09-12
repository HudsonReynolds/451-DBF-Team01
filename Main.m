% this script runs all of the scripts for the aircraft sizing:
clear;clc;close all

% system parameters are all pulled from an Excel spreadsheet. See
% 'SizingParams.xlsx' for more details:

% set plotting up:
set(groot,'defaultLineLineWidth',1.5)
set(groot,'defaultFunctionLineLineWidth',1.5)

% set the interpreter to latex
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter', 'latex');

%change font sizes
set(groot, 'defaultLegendFontSize', 12)
set(groot, 'defaultTextFontSize', 12)
set(groot, 'defaultAxesFontSize', 10)

% grid on and box off
set(groot, 'defaultLegendBox', 'off');
set(groot,'defaultAxesXGrid','on');
set(groot,'defaultAxesYGrid','on');

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