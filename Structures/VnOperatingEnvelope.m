function VnOperatingEnvelope(params)

%% Pull Values From VnDiagram.m

MTOW   = params.performance.MTOW;
S      = params.geometry.S_wing;
CL_max = params.aero.CL_max;
CL_max_neg = params.aero.CL_max_neg;
rho    = params.env.rho;

V_S  = params.performance.V_S;
V_C  = params.performance.V_C;
V_max_level = params.performance.V_max_level;
V_A  = params.performance.V_A;
V_NO = params.performance.V_NO;
V_NE = params.performance.V_NE;
V_D  = params.performance.V_D;

n_pos = params.performance.n_limit_pos;
n_neg = params.performance.n_limit_neg;
n_ult_pos = params.performance.n_ult_pos;
n_ult_neg = params.performance.n_ult_neg;

V_S_neg = sqrt(2*MTOW/(rho*S*abs(CL_max_neg)));
V_G = V_S_neg*sqrt(abs(n_neg));

% Stall boundary curves
V_pos_curve = linspace(V_S, V_A, 200);
n_pos_curve = 0.5*rho*V_pos_curve.^2*S*CL_max/MTOW;

V_neg_curve = linspace(V_G, V_S_neg, 200);
n_neg_curve = -0.5*rho*V_neg_curve.^2*S*abs(CL_max_neg)/MTOW;

V_poly = [V_S, V_pos_curve, V_D,   V_D,   V_G,   V_neg_curve, V_S_neg];
n_poly = [0,   n_pos_curve, n_pos, n_neg, n_neg, n_neg_curve, 0];

%% Plotting

blue    = [0.00 0.45 0.70];
orange  = [0.90 0.35 0.00];
caution = [0.93 0.69 0.13];
danger  = [0.80 0.20 0.20];

x_right = V_D*1.08;
y_top = n_ult_pos + 0.8;
y_bot = n_ult_neg - 0.8;

figure('Name','V-n Envelope: Operating Limits','Color','w','Position',[100 100 950 620],'WindowStyle','docked');
hold on; box on; grid on;
xlim([0, x_right]);
ylim([y_bot, y_top]);

% Operational risk zones, drawn first so the envelope and curves sit on top:
patch([V_NO V_NE V_NE V_NO], [y_bot y_bot y_top y_top], caution, ...
    'FaceAlpha', 0.30, 'EdgeColor', 'none', 'DisplayName', 'caution range ($V_{NO}$-$V_{NE}$)');
patch([V_NE x_right x_right V_NE], [y_bot y_bot y_top y_top], danger, ...
    'FaceAlpha', 0.22, 'EdgeColor', 'none', 'DisplayName', 'beyond $V_{NE}$ (do not fly)');

% Maneuver envelope:
fill(V_poly, n_poly, blue, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'DisplayName', 'maneuver envelope');

% Stall boundary: dotted reference below the stall speed, vertical closure
% at the stall speed, solid from the stall speed to the corner.
plot(linspace(0, V_S, 50), 0.5*rho*linspace(0,V_S,50).^2*S*CL_max/MTOW, ':', 'Color', blue, 'HandleVisibility', 'off');
plot([V_S V_S], [0 n_pos_curve(1)], '-', 'Color', blue, 'LineWidth', 1.5, 'HandleVisibility', 'off');
plot(V_pos_curve, n_pos_curve, '-', 'Color', blue, 'LineWidth', 1.5, 'DisplayName', 'stall boundary ($C_{L,max}$)');

plot(linspace(0, V_S_neg, 50), -0.5*rho*linspace(0,V_S_neg,50).^2*S*abs(CL_max_neg)/MTOW, ':', 'Color', blue, 'HandleVisibility', 'off');
plot([V_S_neg V_S_neg], [0 n_neg_curve(end)], '-', 'Color', blue, 'LineWidth', 1.5, 'HandleVisibility', 'off');
plot(V_neg_curve, n_neg_curve, '-', 'Color', blue, 'LineWidth', 1.5, 'HandleVisibility', 'off');

% Limit load factor lines:
plot([V_A V_D], [n_pos n_pos], '-', 'Color', orange, 'LineWidth', 1.5, ...
    'DisplayName', sprintf('limit load factor %+.2f / %+.2f', n_pos, n_neg));
plot([V_D V_D], [n_neg n_pos], '-', 'Color', orange, 'LineWidth', 1.5, 'HandleVisibility', 'off');
plot([V_G V_D], [n_neg n_neg], '-', 'Color', orange, 'LineWidth', 1.5, 'HandleVisibility', 'off');

% Ultimate load factor lines (1.5x limit, slide 6):
plot([0 x_right], [n_ult_pos n_ult_pos], '--', 'Color', orange, 'LineWidth', 1.1, ...
    'DisplayName', sprintf('ultimate load factor %+.2f (1.5x limit)', n_ult_pos));
plot([0 x_right], [n_ult_neg n_ult_neg], '--', 'Color', orange, 'LineWidth', 1.1, 'HandleVisibility', 'off');

yline(0, 'k-', 'HandleVisibility', 'off');

% Every design speed, labelled, including V_max (the max level speed from
% A6 -- not otherwise drawn on the VnDiagram.m figure):
speed_labels = {'V_S', 'V_A', 'V_C', 'V_{NO}', 'V_{max}', 'V_{NE}', 'V_D'};
speed_vals   = [V_S,   V_A,   V_C,   V_NO,     V_max_level, V_NE,   V_D];
[speed_vals, order] = sort(speed_vals);
speed_labels = speed_labels(order);
row_y = [y_bot + 0.35, y_bot + 0.75];
for i = 1:numel(speed_vals)
    xline(speed_vals(i), ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
    text(speed_vals(i), row_y(mod(i-1,2)+1), sprintf('$%s=%.1f$', speed_labels{i}, speed_vals(i)), ...
        'Color', [0.35 0.35 0.35], 'Interpreter', 'latex', 'FontSize', 9, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end

% V_max gets its own marker+label since it isn't a certification speed
% (it comes from A6/PropulsionSizing, not FAR 23) and can land inside the
% caution range, as here -- worth calling out explicitly.
text(V_max_level, y_top - 0.3, '$V_{max}$ (A6)', 'Interpreter', 'latex', 'FontSize', 9, ...
    'Color', [0.35 0.35 0.35], 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');

xlabel('Equivalent Airspeed, (m/s)');
ylabel('load factor n (g)');
title('V-n Envelope: Maneuver Envelope, Caution Range and Do-Not-Fly Zone');
legend('Location', 'eastoutside');

%exportgraphics(gcf, 'Vn_envelope.png', 'Resolution', 300);

end
