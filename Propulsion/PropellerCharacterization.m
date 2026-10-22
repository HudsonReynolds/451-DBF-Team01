function PropellerCharacterization(params)
%PROPELLERCHARACTERIZATION  Raw APC propeller-table exhibit (A9 Deliverable 1).
%
% Reproduces the two reference figures from the AAE 451 Aircraft
% Performance lecture ("Thrust and Power at Several Shaft Speeds" and
% "APC Data at Several Shaft Speeds"), using the TEAM'S OWN propeller file
% (SizingParams.xlsx prop_file) instead of the lecture's illustrative
% 10x6E example -- same two-panel layout, same per-curve Reynolds-number
% legend. The four shaft speeds are the same throttle settings
% PropulsionSizing.m plots (25/50/75/100% of the motor's max RPM), so the
% curves never go above a speed this motor can actually reach.
%
% This shows the manufacturer's PUBLISHED table data, unmodified. The
% lab-measured static-thrust correction PropulsionDragModel.m applies
% system-wide (A9 Deliverables 1 and 7 -- see that script and
% AssumptionValidation.m item 8) is deliberately NOT applied here: the
% point of this exhibit is to show the underlying APC data itself, the
% thing Deliverable 1 says to state before computing anything else, not
% the team's own corrected flight model.
%
% Run standalone with no arguments (reads SizingParams.xlsx itself), or
% pass an already-loaded params struct to reuse one already in memory.

if nargin < 1 || isempty(params)
    params = readParams('SizingParams.xlsx');
end

IN2M = 0.0254;
MS2MPH = 2.23694;
rho = params.env.rho;
D = IN2M * str2double(regexp(params.prop.prop_file, '_(\d+)x', 'tokens', 'once'));

prop = loadPropData(params.prop.prop_file);

% Shaft speeds as a percentage of the motor's 100% throttle RPM (the
% torque-balance result PropulsionSizing.m stores) -- fixed round speeds
% would plot curves above what the motor can spin the prop to.
if ~isfield(params.prop, 'RPM_max')
    params = PropulsionSizing(params);
end
throttle_settings = params.prop.throttle_settings;
RPM_list = throttle_settings * params.prop.RPM_max;

% Colourblind-safe palette (Wong), with distinct line styles so the plot
% still reads correctly in greyscale -- same figure-standard convention
% used throughout this codebase's other A9 figures.
clr = [0.40 0.40 0.40;    % grey   -- 25%
       0.00 0.45 0.74;    % blue   -- 50%
       0.00 0.60 0.50;    % green  -- 75%
       0.85 0.33 0.10];   % orange -- 100%
sty = {'-', '--', '-.', ':'};
lw = 1.8;

V_sweep = linspace(0, 45, 300); % m/s, comfortably covers J up to ~0.75 at every RPM above
V_mph = MS2MPH * V_sweep;

%% ---- Figure 1: thrust and shaft power vs airspeed, at each shaft speed ----
figure('Name', 'APC Propeller Data -- Thrust and Power vs Airspeed', 'Color', 'w', 'WindowStyle', 'docked');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on; box on; grid on;
T_static = nan(1, numel(RPM_list));
for i = 1:numel(RPM_list)
    RPM = RPM_list(i);
    n = RPM/60;
    r = prop.query(RPM, V_mph);
    T = r.Ct * rho * n^2 * D^4;
    T(~r.inRange) = NaN;
    T_static(i) = T(1); % V=0
    plot(V_sweep, T, 'Color', clr(i,:), 'LineStyle', sty{i}, 'LineWidth', lw, ...
        'DisplayName', sprintf('%.0f\\%% (%.0f rpm)', 100*throttle_settings(i), RPM)); % \% because legends use the LaTeX interpreter (Setup.m), where a bare % starts a comment
end
xlabel('Airspeed V (m/s)'); ylabel('Thrust (N)');
title('Thrust, T = CT x rho x n^2 x D^4', 'Interpreter', 'none');
legend('Location', 'best');

nexttile; hold on; box on; grid on;
P_static = nan(1, numel(RPM_list));
for i = 1:numel(RPM_list)
    RPM = RPM_list(i);
    n = RPM/60;
    r = prop.query(RPM, V_mph);
    P = r.Cp * rho * n^3 * D^5;
    P(~r.inRange) = NaN;
    P_static(i) = P(1); % V=0
    plot(V_sweep, P, 'Color', clr(i,:), 'LineStyle', sty{i}, 'LineWidth', lw, ...
        'DisplayName', sprintf('%.0f\\%% (%.0f rpm)', 100*throttle_settings(i), RPM)); % \% because legends use the LaTeX interpreter (Setup.m), where a bare % starts a comment
end
xlabel('Airspeed V (m/s)'); ylabel('Shaft power (W)');
title('Shaft power, Pshaft = CP x rho x n^3 x D^5', 'Interpreter', 'none');
legend('Location', 'best');

sgtitle(sprintf('Thrust and Power at Several Shaft Speeds -- %s', params.prop.prop_file), 'FontWeight', 'bold', 'Interpreter', 'none');

%% ---- Figure 2: CT and CP vs advance ratio, at each shaft speed ----
figure('Name', 'APC Propeller Data -- CT and CP vs Advance Ratio', 'Color', 'w', 'WindowStyle', 'docked');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

Re_list = nan(1, numel(RPM_list));
nexttile; hold on; box on; grid on;
for i = 1:numel(RPM_list)
    RPM = RPM_list(i);
    r = prop.query(RPM, V_mph);
    r0 = prop.query(RPM, 0);
    Re_list(i) = r0.Reyn;
    Ct = r.Ct; Ct(~r.inRange) = NaN;
    plot(r.J, Ct, 'Color', clr(i,:), 'LineStyle', sty{i}, 'LineWidth', lw, ...
        'DisplayName', sprintf('%.0f%% (%.0f rpm), Re %.0fk at 0.75R', 100*throttle_settings(i), RPM, Re_list(i)/1000));
end
xlabel('Advance ratio J'); ylabel('CT (-)');
title('Thrust coefficient');
legend('Location', 'best', 'Interpreter', 'none');

nexttile; hold on; box on; grid on;
for i = 1:numel(RPM_list)
    RPM = RPM_list(i);
    r = prop.query(RPM, V_mph);
    Cp = r.Cp; Cp(~r.inRange) = NaN;
    plot(r.J, Cp, 'Color', clr(i,:), 'LineStyle', sty{i}, 'LineWidth', lw, ...
        'DisplayName', sprintf('%.0f%% (%.0f rpm), Re %.0fk at 0.75R', 100*throttle_settings(i), RPM, Re_list(i)/1000));
end
xlabel('Advance ratio J'); ylabel('CP (-)');
title('Power coefficient');
legend('Location', 'best', 'Interpreter', 'none');

sgtitle(sprintf('APC Data at Several Shaft Speeds -- %s', params.prop.prop_file), 'FontWeight', 'bold', 'Interpreter', 'none');

%% ---- Console summary, same style as the lecture's own figure captions ----
fprintf('\n--- APC Propeller Data (%s, D=%.1f in) ---\n', params.prop.prop_file, D/IN2M);
fprintf('  Static thrust/power at each shaft speed (V=0):\n');
for i = 1:numel(RPM_list)
    fprintf('    %3.0f%% (%5.0f rpm): T = %6.2f N   P = %6.1f W   Re = %.0fk at 75%% radius\n', ...
        100*throttle_settings(i), RPM_list(i), T_static(i), P_static(i), Re_list(i)/1000);
end
i2x = find(RPM_list == 2*RPM_list(1), 1); % the one pair in RPM_list that's an exact doubling, if any
if ~isempty(i2x)
    fprintf('  Doubling n (%.0f -> %.0f rpm): static thrust scales %.2fx, power scales %.2fx (ideal: 4x, 8x).\n', ...
        RPM_list(1), RPM_list(i2x), T_static(i2x)/T_static(1), P_static(i2x)/P_static(1));
end
fprintf('  The blocks differ because the Reynolds number changes with shaft speed -- not a fixed-efficiency propeller.\n');

end
