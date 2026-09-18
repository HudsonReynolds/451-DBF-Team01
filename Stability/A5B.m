function A5B(params)

% top level control plot

params = AircraftScissorPlot(params);
params = TrimAircraft(params); 
[outputs, params] = ControlSurfaceSizing(params);
params = StabilityDerivatives(params);

end