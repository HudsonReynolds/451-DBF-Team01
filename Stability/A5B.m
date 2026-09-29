function params = A5B(params)

% top level control plot

params = AircraftScissorPlot(params);
params = TrimAircraft(params);
[~, params] = ControlSurfaceSizing(params);
StabilityDerivatives(params); % returns a summary struct only, not the full params -- must not overwrite params here

end