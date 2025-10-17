function OptimizeTrajectoryViaLIOM()
global params_
params_.limo.is_successful = 0;

x = params_.ha_result.x;
y = params_.ha_result.y;
theta = params_.ha_result.theta;
v = params_.ha_result.v;
a = params_.ha_result.a;
phy = params_.ha_result.phy;
w = params_.ha_result.w;
terminal_time = params_.ha_result.terminal_time;
FormInitialGuessViaFullConfig(x, y, theta, v, a, phy, w, terminal_time);
sol_prev = [x, y, theta];
params_.task.thetaf = params_.ha_result.theta(end);
WriteBoundaryValues();

DrawParkingScenario();
plot(params_.ha_result.x, params_.ha_result.y, 'r:', 'LineWidth', 2); hold on; drawnow;

iter = 0;
while (1)
    iter = iter + 1; disp(['LIOM Iter = ', num2str(iter)]);

    xr = x + params_.vehicle.r2p .* cos(theta);
    yr = y + params_.vehicle.r2p .* sin(theta);
    xf = x + params_.vehicle.f2p .* cos(theta);
    yf = y + params_.vehicle.f2p .* sin(theta);

    ConstructSafeTravelCorridors(xr, yr, 'rear');
    ConstructSafeTravelCorridors(xf, yf, 'fron');

    WriteBasicParameterFile();
    !ampl rr.run
    [x, y, theta, v, a, phy, w, terminal_time, infeasibility] = LoadAmplSolution();
    plot(x, y, 'k', 'LineWidth', 1); hold on; drawnow;

    sol = [x, y, theta];
    diff = norm(sol - sol_prev);

    if (infeasibility < params_.opti.feasibility_tolerance)&&(diff < params_.opti.diff_tolerance)
        params_.limo.is_successful = 1;
        params_.limo.x = x;
        params_.limo.y = y;
        params_.limo.theta = theta;
        params_.limo.phy = phy;
        params_.limo.v = v;
        params_.limo.a = a;
        params_.limo.w = w;
        params_.limo.terminal_time = terminal_time;
        plot(params_.limo.x, params_.limo.y, 'b', 'LineWidth', 2); hold on; drawnow;

        PlotOptimalProfiles();
        return;
    end
    params_.opti.cost_function_external_penalty_weight = params_.opti.cost_function_external_penalty_weight * 4.0;
    params_.opti.cost_function_external_penalty_weight = min(params_.opti.cost_function_external_penalty_weight, 1e9);
    sol_prev = sol;
end
end