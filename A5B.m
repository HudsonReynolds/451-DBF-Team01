function A5B(data)

% top level control plot

data = AircraftScissorPlot(data);

data = TrimAircraft(data); 
data = ControlSurfaceSizing(data);
data = StabilityDerivatives(data);

end