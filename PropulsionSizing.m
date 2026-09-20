function PropulsionSizing(params)

IN2M = .0254;
MS2MPH = 2.23694;

%% Thrust
L_TO = 0.5 * params.env.rho * (0.7*params.performance.V_TO)^2 * params.geometry.S_wing * params.aero.CL_R % Lift at takeoff [N]
D_TO = 0.5 * params.env.rho * (0.7*params.performance.V_TO)^2 * params.geometry.S_wing * (params.aero.CD_0 + params.aero.K_wing*params.aero.CL_R^2) % Drag at takeoff [N]
T_static = (params.performance.MTOW * params.performance.V_TO^2) / (2 * params.env.g * params.performance.S_TO) + D_TO + params.env.mu_TO*(params.performance.MTOW - L_TO) % Static thrust [N]
T_C = 0.5 * params.env.rho * params.performance.V_C^2 * params.geometry.S_wing * (params.aero.CD_0 + params.aero.K_wing*params.aero.CL_C^2) % Cruise thrust [N]

%% Power
eta_motor = 0.8; % Motor efficiency
eta_ESC = 0.95; % ESC efficiency

P_shaft_total = params.performance.MTOW / params.performance.W_P_design; % Total shaft power [W]
P_shaft = P_shaft_total / 2; % Shaft power per motor [W]
P_motor = P_shaft / eta_motor % Motor power per motor [W]
P_battery = P_motor / eta_ESC % Battery power per motor [W]

%% System specs
Kv = 1050; % [RPM/V]
Kt = 60 / (2*pi*Kv); % [N-m/A]
I0 = 0.9; % No load current [A]
V = 14.8; % Motor voltage [V]
R = 0.045; % Motor resistance [Ohms]


%% Analysis
filename = "PER3_9x45E.txt";
prop = loadPropData(filename);

D = IN2M * str2double(regexp(filename, '_(\d+)x', 'tokens', 'once')); % Prop diameter [m]
V_range = linspace(0, 25); % Airspeed range [m/s]

% Torque balance
RPM_noload = V * Kv; % Max motor speed [RPM]
RPM_high = min(RPM_noload, 25000); % Max of RPM range for solver [RPM]

Qres = @(RPM) max(Kt*((V - Kt*(2*pi/60)*RPM)/R - I0), 0) ...            % Q_motor
             - prop.query(RPM,0).Cp * params.env.rho * (RPM/60)^2 * D^5 / (2*pi);  % Q_prop

RPM_eq = fzero(Qres, [1000, RPM_high])

% Get prop data
RPM_max = RPM_eq;
throttle_setting = [0.25 0.5 0.75 1];

T = zeros(numel(throttle_setting), numel(V_range));
P = zeros(numel(throttle_setting), numel(V_range));
J = zeros(numel(throttle_setting), numel(V_range));
eta = zeros(numel(throttle_setting), numel(V_range));

for i = 1:numel(throttle_setting)
    RPM = throttle_setting(i) * RPM_max;
    n = RPM / 60;
    r = prop.query(RPM, MS2MPH * V_range);

    J(i,:) = V_range ./ (n .* D);
    T(i,:) = 2 .* r.Ct .* params.env.rho .* n.^2 .* D.^4;
    P(i,:) = 2 .* r.Cp .* params.env.rho .* n.^3 .* D.^5;
    eta(i,:) = (J(i,:) .* r.Ct) ./ r.Cp;
end

% Plots
clr = [0.00 0.45 0.74;   % blue   – 25%
       0.20 0.63 0.47;   % green  – 50%
       0.93 0.69 0.13;   % gold   – 75%
       0.85 0.33 0.10];  % orange – 100%
sty = {'-','--','-.',':'};
lw  = 1.5;

figure('Color','w','Position',[100 100 1200 340]);
tiledlayout(1,3,'TileSpacing','compact','Padding','compact');

% Thrust
nexttile; hold on; box on; grid on;
for i = 1:4
    plot(V_range, T(i,:), 'Color', clr(i,:), 'LineStyle', sty{i}, 'LineWidth', lw);
end
xlabel('Airspeed [m/s]'); ylabel('Thrust [N]'); title('Thrust vs. Airspeed');

% Electrical power
nexttile; hold on; box on; grid on;
for i = 1:4
    plot(V_range, P(i,:), 'Color', clr(i,:), 'LineStyle', sty{i}, 'LineWidth', lw);
end
xlabel('Airspeed [m/s]'); ylabel('Electrical power [W]'); title('Electrical Power vs. Airspeed');

% Efficiency
nexttile; hold on; box on; grid on;
h = gobjects(1,4);
for i = 1:4
    h(i) = plot(V_range, eta(i,:), 'Color', clr(i,:), 'LineStyle', sty{i}, 'LineWidth', lw);
end
ylim([0 1]);
xlabel('Airspeed [m/s]'); ylabel('Propeller efficiency [-]'); title('Efficiency vs. Airspeed');
legend(h, {'25%','50%','75%','100%'}, 'Location','northwest');