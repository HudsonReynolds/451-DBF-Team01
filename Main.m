% this script runs all of the scripts for the aircraft sizing:

% system parameters are all pulled from an Excel spreadsheet. See
% 'SizingParams.xlsx' for more details:

set(groot, 'defaultAxesTickLabelInterpreter','latex')

% load the values from the excel sheet
params = readcell("SizingParams.xlsx");

N = length(params);

% put the data into a MATLAB 
for idx = 2:N
    data.(params{idx,1}) = params{idx,2};
end


% run all of the sizing:
InitialVehicleSizingConstraint(data);
 
VehicleWeightEstimation(data);

AircraftScissorPlot(data);