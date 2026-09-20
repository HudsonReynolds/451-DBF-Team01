function PropulsionSizing(params)

IN2M = .0254;
MS2MPH = 2.23694;

%% Initial Sizing
L_TO = 0.5 * params.env.rho * (0.7*params.performance.V_TO)^2 * params.geometry.S_wing * params.aero.CL_R % Lift at takeoff [N]
D_TO = 0.5 * params.env.rho * (0.7*params.performance.V_TO)^2 * params.geometry.S_wing * (params.aero.CD_0 + params.aero.K_wing*params.aero.CL_R^2) % Drag at takeoff [N]
T_static = (params.performance.MTOW * params.performance.V_TO^2) / (2 * params.env.g * params.performance.S_TO) + D_TO + params.env.mu_TO*(params.performance.MTOW - L_TO) % Static thrust [N]
T_C = 0.5 * params.env.rho * params.performance.V_C^2 * params.geometry.S_wing * (params.aero.CD_0 + params.aero.K_wing*params.aero.CL_C^2) % Cruise thrust [N]

eta_motor = 0.8; % Motor efficiency
eta_ESC = 0.95; % ESC efficiency

P_shaft_total = params.performance.MTOW / params.performance.W_P_design; % Total shaft power [W]
P_shaft = P_shaft_total / 2; % Shaft power per motor [W]
P_motor = P_shaft / eta_motor % Motor power per motor [W]
P_battery = P_motor / eta_ESC % Battery power per motor [W]

%% System specs
Kv = 1170; % [RPM/V]
Kt = 60 / (2*pi*Kv); % [N-m/A]
I0 = 1.6; % No load current [A]
V = 14.8; % Motor voltage [V]
R = 0.027; % Motor resistance [Ohms]

%% Analysis
% Load prop data
filename = "PER3_8x4E.txt";
prop = loadPropData(filename);

D = IN2M * str2double(regexp(filename, '_(\d+)x', 'tokens', 'once')); % Prop diameter [m]
V_range = linspace(0, 40); % Airspeed range [m/s]

% Drag
CL_level = params.performance.MTOW ./ (0.5 * params.env.rho * V_range.^2 * params.geometry.S_wing);
D_level = 0.5 * params.env.rho .* V_range.^2 .* params.geometry.S_wing .* (params.aero.CD_0 + params.aero.K_wing*CL_level.^2);

% Torque balance
RPM_noload = V * Kv; % Max motor speed [RPM]
RPM_high = min(RPM_noload, 25000); % Max of RPM range for solver [RPM]

Qres = @(RPM) max(Kt*((V - Kt*(2*pi/60)*RPM)/R - I0), 0) ...            % Q_motor
             - prop.query(RPM,0).Cp * params.env.rho * (RPM/60)^2 * D^5 / (2*pi);  % Q_prop

RPM_eq = fzero(Qres, [1000, RPM_high]) % Equilibrium motor speed [RPM]

% Determine motor operating speeds
RPM_lim = 145000 / (D / IN2M) % Propellor structural speed limit
RPM_max = min(RPM_eq, RPM_lim) % 100% throttle RPM
throttle_settings = [0.25 0.5 0.75 1];

% Get propellor data
Ct = zeros(numel(throttle_settings), numel(V_range));
Cp = zeros(numel(throttle_settings), numel(V_range));
J  = zeros(numel(throttle_settings), numel(V_range));

for i = 1:numel(throttle_settings)
    r = prop.query(throttle_settings(i)*RPM_max, MS2MPH*V_range);
    J(i,:)  = r.J;
    Ct(i,:) = r.Ct;
    Cp(i,:) = r.Cp;  
end

% System calculations
RPM = throttle_settings(:) * RPM_max;
n = RPM / 60;
omega = 2 * pi * n;

T = Ct .* params.env.rho .* n.^2 .* D^4; % Thrust [N]    
P_shaft = Cp .* params.env.rho .* n.^3 .* D^5; % Shaft power [W]
eta_prop = (J .* Ct) ./ Cp; % Propellor efficiency [-]
                     
I = P_shaft ./ (Kt * omega) + I0; % Motor current [A]
P_elec = (omega.*Kt.*I + I.^2*R) ./ eta_ESC; % Electrical power from battery [W]

T_total = 2 * T; % Total vehicle thrust [N]
P_elec_total = 2 * P_elec; % Total vehicle battery power [W]

% Find max level airspeed
diff = T_total(4,:) - D_level;
good = ~isnan(diff);
V_max = interp1(diff(good), V_range(good), 0);
T_V_max = interp1(V_range, T_total(4,:), V_max);

%% Plots
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
    plot(V_range, T_total(i,:), 'Color', clr(i,:), 'LineStyle', sty{i}, 'LineWidth', lw);
end
plot(V_range, D_level, 'Color', 'k');

plot(0, T_static, 'r*');
text(0.5, T_static-1, 'Required static thrust', 'FontSize', 10, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');

plot(params.performance.V_C, T_C, 'r*');
text(params.performance.V_C-1, T_C-1, 'Required cruise thrust', 'FontSize', 10, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');

plot(V_max, T_V_max, 'r*');
text(V_max-0.5, T_V_max+0.5, 'Max level speed', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right')

xlabel('Airspeed [m/s]'); ylabel('Thrust [N]'); title('Thrust vs. Airspeed'); ylim([0, 35]);
legend({'25%','50%','75%','100%', 'Drag'}, 'Location','northeast');

% Electrical power
nexttile; hold on; box on; grid on;
for i = 1:4
    plot(V_range, P_elec_total(i,:), 'Color', clr(i,:), 'LineStyle', sty{i}, 'LineWidth', lw);
end
xlabel('Airspeed [m/s]'); ylabel('Electrical power [W]'); title('Electrical Power vs. Airspeed');

% Efficiency
nexttile; hold on; box on; grid on;
for i = 1:4
    plot(V_range, eta_prop(i,:), 'Color', clr(i,:), 'LineStyle', sty{i}, 'LineWidth', lw);
end
ylim([0 1]);
xlabel('Airspeed [m/s]'); ylabel('Propeller efficiency [-]'); title('Efficiency vs. Airspeed');
