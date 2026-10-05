function params = A5B(params)

% top level control plot

params = AircraftScissorPlot(params);
params = TrimAircraft(params);
[csOutputs, params] = ControlSurfaceSizing(params);
sdOutputs = StabilityDerivatives(params); % returns a summary struct only, not the full params -- must not overwrite params here

% Exposed for reuse (e.g. Deliverable 8's payload-sweep logging) -- both
% of these were previously computed and printed but then discarded here.
params.stability.lastControlSurfaces = csOutputs.ControlSurfaces;
params.stability.lastDerivatives = sdOutputs.StabilityDerivatives;

end