function [Result, input_param] = GA(terr)
% Runs a genetic algorithm for a given environment

    %% Tuning Parameters
    N = terr.n;
    max_sensors = 10;
    preset_dir = 4;
    max_range = 10;
    max_el_acc = .95;
    max_obs_acc = .99;
    min_range = 4;
    min_el_acc = 0.4; 
    min_obs_acc = 0.4;
    cost_range = 1;
    cost_acc = 2;
    beta = 1;
    size_pop = 20;
    
    i_max_sensors = 1;
    i_range = [i_max_sensors + 1, i_max_sensors + max_sensors];
    i_direction = [i_range(2) + 1, i_range(2) + max_sensors - preset_dir];
    i_el_acc = [i_direction(2) + 1, i_direction(2) + max_sensors];
    i_obs_acc = [i_el_acc(2) + 1, i_el_acc(2) + max_sensors];
    
    max_sensor_cost = max_sensors * sensorCost(max_range, max_el_acc, max_obs_acc, cost_range, cost_acc);
    best_path_cost = 2*N;
    
    input_param = [cost_range, cost_acc, max_sensor_cost, best_path_cost, beta,...
        i_max_sensors, i_range, i_direction, i_el_acc, i_obs_acc];
    
    x0 = [max_sensors,...
          max_range * ones(1, max_sensors),...
          randi(4, [1, max_sensors - preset_dir]),...
          max_el_acc * ones(1, max_sensors),...
          max_obs_acc * ones(1, max_sensors)];
    
    lb = [preset_dir,...
         min_range * ones(1, max_sensors),...
         ones(1, max_sensors - preset_dir),...
         min_el_acc * ones(1, max_sensors),...
         min_obs_acc * ones(1, max_sensors)];
    
    ub = [max_sensors,...
          max_range * ones(1, max_sensors),...
          randi(4, [1, max_sensors - preset_dir]),...
          max_el_acc * ones(1, max_sensors),...
          max_obs_acc * ones(1, max_sensors)];
    
    intcon = 1:i_max_sensors + max_sensors + (max_sensors - preset_dir);
    options = optimoptions( 'ga', 'PopulationSize', 200, ...
        'FunctionTolerance', 0.001, 'PlotFcn', {@gaplotbestf, @gaplotrange},...
        'MaxGenerations',500, 'MaxStallGenerations', 50, ...
        'InitialPopulationMatrix', x0); 

    [Result.X, Result.Fval, Result.GAexitflag, Result.GAoutput, Result.GApopulation, Result.GAscores] = ...
        ga(@(X)obj_fun(X, terr, input_param), length(x0), [], [], [], [], lb, ub,[],intcon, options);
end

function [cost] = obj_fun(X, terr, input_param)
    rob = decodeGenome(X, terr, input_param);
    rob.pathfind();
   
    cost = totalCost(rob.sensorCost(), ...
        rob.costOfPath(input_param(5)), input_param(3), input_param(4));
end

function [cost] = sensorCost(range, el_acc, obs_acc, c1, c2)
    cost = c1 * range + c2 * el_acc + c2 * obs_acc;
end
