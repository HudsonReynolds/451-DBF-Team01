function InitialVehicleSizingConstraint(params)
    %% First Constraint: Stall Speed
    % this determines the range of values to consider for W/S:
    W_S_max = 0.5*params.env.rho*params.aero.V_S^2*params.aero.CL_max; 
    W_S = 0:W_S_max;
    
    %% Second Constraint: Cruise Speed
    W_P_cruise = (params.prop.eta_p*params.performance.phi_C ...
        / (0.5*params.aero.CD_cruise*params.env.rho*params.performance.V_C^3)) * W_S;
    
    %% Third Constraint: Climb Requirement
    % climb at 0.866 L/D max:
    L_D_climb = 0.866*params.aero.L_D_max;
    W_P_climb = params.prop.eta_p / (params.aero.V_S * ...
        (1 / L_D_climb + sin((pi/180) * params.performance.gamma)));
    
    %% Fourth Constraint: Maneuver Requirement
    q_m = 0.5 * params.env.rho * params.performance.V_M^2;

    W_P_m = params.prop.eta_p ./ (q_m*params.performance.V_M*...
    (params.aero.CD_0./W_S + params.aero.K_wing*(params.performance.n/q_m)^2*W_S));
    
    %% Fifth Constaint: Takeoff Requirement
    numer = 1 - exp(0.6*params.env.rho*params.env.g*params.aero.CD_G*...
        params.performance.S_TO*(1./W_S));
    denom = params.env.mu_TO-(params.env.mu_TO+params.aero.CD_G/params.aero.CL_R) ...
        * (exp(0.6*params.env.rho*params.env.g*params.aero.CD_G*params.performance.S_TO*1./W_S));
    W_P_TO = (numer ./ denom) * (params.prop.eta_p_TO / params.performance.V_TO);
    
    
    %% Final constraint plot:
    % create a filter to show the feasible region
    W_P_climb_vec = W_P_climb * ones(size(W_S));
    lower_envelope = min([W_P_climb_vec(:), W_P_cruise(:), W_P_m(:), W_P_TO(:)], [], 2);
    
    mask = (W_S >= 0) & (W_S <= W_S_max);
    
    x_fill = W_S(mask);
    x_fill = x_fill(:);
    y_fill = lower_envelope(mask);
    
    figure('Name','Aicraft Constraint Diagram');
    
    % Fill region down to y = 0
    fill([x_fill; flipud(x_fill)], [y_fill; zeros(size(y_fill))], [0.8 0.6 0.7], ...
        'FaceAlpha', 0.4, 'EdgeColor', 'none', 'DisplayName', 'Feasible Region');
    hold on;
    
    % Plot constraint lines
    plot(W_S_max*ones(2,1),[0;0.25], 'r-', 'DisplayName', 'Stall Speed');
    plot([0;W_S_max],W_P_climb*ones(2,1), 'g--', 'DisplayName', 'Climb Constraint');
    plot(W_S,W_P_cruise, 'b:', 'DisplayName', 'Cruise Constraint');
    plot(W_S,W_P_m, 'c-.', 'DisplayName', 'Maneuver Constraint');
    plot(W_S,W_P_TO, 'k-',  'LineWidth', 2.5, 'DisplayName', 'Takeoff Constraint');
    plot(params.performance.W_S_design,params.performance.W_P_design, ...
        'r*', 'MarkerSize', 12, 'DisplayName', 'Design Point')
    text(params.performance.W_S_design-1, params.performance.W_P_design-.001, ...
        sprintf('(%.1f, %.3f)', params.performance.W_S_design, params.performance.W_P_design), ...
        'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'Color', 'r')
    
    ylim([0, 0.50]);
    xlim([0, W_S_max*1.05]);
    set(gca, 'FontSize', 14);
    xlabel("$\frac{W}{S} \left[\frac{N}{m^2}\right]$")
    ylabel("$\frac{W}{P} \left[\frac{N}{W}\right]$", 'Rotation', 0)
    title('Aircraft Constraint Diagram')
    legend('Location','northeast')

end





