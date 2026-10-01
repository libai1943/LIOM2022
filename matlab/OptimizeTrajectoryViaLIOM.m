function result = OptimizeTrajectoryViaLIOM()
% Algorithm 5-1: rebuild corridors, solve, update, and check BOTH criteria.
global params_
initial = CompleteInitialGuess(params_.ha_result);
previous = initial;
params_.task.thetaf = initial.theta(end);
WriteBoundaryValues();
history = {}; statistics = [];
for iter = 1 : params_.opti.max_iterations
    g = params_.vehicle;
    ConstructSafeTravelCorridors(previous.x+g.r2p*cos(previous.theta), ...
        previous.y+g.r2p*sin(previous.theta),'rear');
    ConstructSafeTravelCorridors(previous.x+g.f2p*cos(previous.theta), ...
        previous.y+g.f2p*sin(previous.theta),'fron');
    WriteInitialGuess(previous);
    WriteBasicParameterFile();
    current = SolveIntermediateProblem();
    difference = norm(SolutionVector(current)-SolutionVector(previous),2);
    weight = params_.opti.cost_function_external_penalty_weight;
    history{end+1} = current;
    statistics(end+1,:) = [iter,weight,current.cost,current.infeasibility,difference];
    fprintf('LIOM %2d | weight %.3g | J %.7f | infeasibility %.3g | difference %.4g\n', ...
        iter,weight,current.cost,current.infeasibility,difference);
    if current.infeasibility < params_.opti.feasibility_tolerance && difference < params_.opti.diff_tolerance
        result = current;
        result.initial_guess = initial;
        result.history = history;
        result.iterations = statistics;
        result.rear_corridor = params_.opti.stc.rear_stc;
        result.front_corridor = params_.opti.stc.front_stc;
        result.difference = difference;
        result.is_successful = true;
        result.validation = ValidateSolution(result);
        params_.limo = result;
        return;
    end
    previous = current;
    params_.opti.cost_function_external_penalty_weight = weight*params_.opti.penalty_multiplier;
end
error('LIOM:NotConverged','Both stopping criteria were not met within %d iterations.',params_.opti.max_iterations);
end
