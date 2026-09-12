function DragPolars(data)

% Plot clean and trimmed drag polars
CL = linspace(0, data.CL_max);
CD = data.CD_0 + data.K_wing.*CL.^2;

plot(CD, CL);
grid on;
xlabel("C_D");
ylabel("C_L");
title("Clean Drag Polar");

end