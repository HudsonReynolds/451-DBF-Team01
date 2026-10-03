function [outputs, params] = TurnPerformance(params)

%% A9 Deliverable 5 -- Turn
%
% At the COURSE turn radius (performance.R_req) -- not performance.R_M,
% the separate design radius InitialCalcs.m used to size the maneuver
% speed V_M and the structural load factor n=1.5 in the first place.
% Flies that course circle at the existing V_M (reused, since nothing
% else in the project defines a different speed for the laps) and
% recomputes the load factor and bank angle AT R_req fresh, rather than
% assuming they equal the A2-era design values -- if R_req differs from
% R_M, they will legitimately differ too, and that comparison is itself
% a useful check on whether the structural design load factor (used
% throughout A8) still matches what the course actually demands.

if ~isfield(params.prop, 'model')
    error('TurnPerformance:noModel', 'Run PropulsionDragModel(params) first.');
end
model = params.prop.model;

rho = params.env.rho;
S = params.geometry.S_wing;
MTOW = params.performance.MTOW;
g = params.env.g;

V_M = params.performance.V_M;
R_req = params.performance.R_req;

n_turn = sqrt(1 + (V_M^2/(g*R_req))^2);
bank_turn_deg = acosd(1/n_turn);

fprintf('\n--- Turn Performance (Deliverable 5) ---\n');
fprintf('  Course turn radius R_req = %.1f m (NOT the design radius R_M = %.1f m used to size V_M/n in A2)\n', R_req, params.performance.R_M);
fprintf('  At V_M = %.2f m/s: bank = %.1f deg, load factor n = %.2f\n', V_M, bank_turn_deg, n_turn);
if abs(n_turn - params.performance.n) > 0.05
    fprintf('  >>> FINDING: load factor at the course radius (%.2f) differs from the structural design n=%.2f used throughout A8 -- worth reconciling R_M against R_req.\n', ...
        n_turn, params.performance.n);
end

%% ---- Power required at this turn vs. power available ----
q = 0.5*rho*V_M^2;
CL_turn = n_turn*MTOW/(q*S);
stall_margin_ok = CL_turn <= params.aero.CL_max;
CD_turn = model.CD_trim_fn(CL_turn);
D_turn = q*S*CD_turn;
T_req_turn = D_turn; % coordinated level turn: thrust balances drag, same as level flight
P_req_aero_turn = D_turn * V_M;

T_avail_turn = model.T_avail_fn(V_M);
P_avail_elec_turn = model.Pelec_avail_fn(V_M);

if isnan(T_avail_turn) || T_req_turn > T_avail_turn
    P_req_elec_turn = NaN;
    ok_power = false;
else
    tau_turn = model.find_throttle_for_thrust(V_M, T_req_turn);
    if ~isnan(tau_turn)
        P_req_elec_turn = model.Pelec_grid_fn(tau_turn, V_M);
        ok_power = P_req_elec_turn <= P_avail_elec_turn;
    else
        P_req_elec_turn = NaN;
        ok_power = false;
    end
end

fprintf('  CL in the turn = %.3f (CL_max = %.3f) -- %s\n', CL_turn, params.aero.CL_max, local_ternary(stall_margin_ok, 'within stall margin', 'EXCEEDS CL_max, turn not achievable as modelled'));
fprintf('  Power required: %.1f W electrical (%.1f W aerodynamic) vs. %.1f W available at %.2f m/s -- %s\n', ...
    P_req_elec_turn, P_req_aero_turn, P_avail_elec_turn, V_M, local_ternary(ok_power && stall_margin_ok, 'PASS', 'FAIL'));

%% ---- Outputs ----
outputs.TurnPerformance = struct('R_req', R_req, 'V_M', V_M, 'n_turn', n_turn, 'bank_turn_deg', bank_turn_deg, ...
    'CL_turn', CL_turn, 'stall_margin_ok', stall_margin_ok, ...
    'P_req_aero_turn', P_req_aero_turn, 'P_req_elec_turn', P_req_elec_turn, 'P_avail_elec_turn', P_avail_elec_turn, ...
    'ok_power', ok_power && stall_margin_ok);

params.performance.n_turn = n_turn;
params.performance.bank_turn_deg = bank_turn_deg;
params.performance.turn_ok = ok_power && stall_margin_ok;

end

function out = local_ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
