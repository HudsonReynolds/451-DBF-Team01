clear;
clc;

params = readcell("SizingParams.xlsx");

N = length(params);

for idx = 2:N
    data.(params{idx,1}) = params{idx,2};
end

%% Thrust
L_TO = 0.5 * data.rho * (0.7*data.V_TO)^2 * data.S_wing * data.CL_C; % Lift at takeoff [N]
D_TO = 0.5 * data.rho * (0.7*data.V_TO)^2 * data.S_wing * (data.CD_0 + data.K_wing * data.CL_C^2); % Drag at takeoff [N]
T_static = (data.MTOW * data.V_TO^2) / (2 * data.g * data.S_TO) + D_TO + data.mu_TO*(data.MTOW - L_TO) % Static thrust [N]
T_C = 0.5 * data.rho * data.V_C^2 * data.S_wing * (data.CD_0 + data.K_wing * data.CL_C^2) % Cruise thrust [N]

%% Power
eta_motor = 0.8; % Motor efficiency
eta_ESC = 0.95; % ESC efficiency

P_shaft_total = data.MTOW / data.W_P_design; % Total shaft power [W]
P_shaft = P_shaft_total / 2; % Shaft power per motor [W]
P_motor = P_shaft / eta_motor; % Motor power per motor [W]
P_battery = P_motor / eta_ESC; % Battery power per motor [W]

%% Analysis
% filename = "PER3_8x6.txt";
% prop = loadPropData(filename);
% 
% D = str2double(regexp(filename, '_(\d+)x', 'tokens', 'once')) * .0254;
% V_range = linspace(0, 25);
% RPM_max = 10000;
% throttle_setting = [0.25 0.5 0.75 1];
% 
% T = zeros(numel(throttle_setting), numel(V_range));
% P = zeros(numel(throttle_setting), numel(V_range));
% J = zeros(numel(throttle_setting), numel(V_range));
% eta = zeros(numel(throttle_setting), numel(V_range));
% 
% for i = 1:numel(throttle_setting)
%     RPM = throttle_setting(i) * RPM_max;
%     n = RPM / 60;
%     r = prop.query(RPM, V_range*2.23694);
% 
%     J(i,:) = V_range ./ (n .* D);
%     T(i,:) = r.Ct .* data.rho .* n.^2 .* D.^4;
%     P(i,:) = r.Cp .* data.rho .* n.^3 .* D.^5;
%     eta(i,:) = (J(i,:) .* r.Ct) ./ r.Cp;
% end
% 
% figure(1);
% plot(V_range, T(1,:), V_range, T(2,:), V_range, T(3,:), V_range, T(4,:));
% 
% figure(2);
% plot(V_range, P(1,:), V_range, P(2,:), V_range, P(3,:), V_range, P(4,:));
% 
% figure(3);
% plot(V_range, eta(1,:), V_range, eta(2,:), V_range, eta(3,:), V_range, eta(4,:));


