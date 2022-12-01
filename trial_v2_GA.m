clc;

rng(30);

%% Now let us set up the terrain
N = 20;
max_height = 3;
steepness = 0.25;
density = 0.2;
terr = Terrain(N, max_height, steepness, density);
disp(terr.toString());

% Tuning Parameters
max_sensors = 10;
preset_dir = 4;
max_range = 10;
max_el_acc = .95;
max_obs_acc = .99;
min_el_acc = 0;
min_obs_acc = 0;
cost_range = 1;
cost_acc = 2;
beta = 1;

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
    ones(1, max_sensors),...
    ones(1, max_sensors - preset_dir),...
    min_el_acc * ones(1, max_sensors),...
    min_obs_acc * ones(1, max_sensors)];

ub = [max_sensors,...
      max_range * ones(1, max_sensors),...
      randi(4, [1, max_sensors - preset_dir]),...
      max_el_acc * ones(1, max_sensors),...
      max_obs_acc * ones(1, max_sensors)];

intcon = 1:i_max_sensors + max_sensors + (max_sensors - preset_dir);
options = optimoptions( 'ga', 'PopulationSize', 20, 'FunctionTolerance', 0.01, 'PlotFcn', {@gaplotbestf, @gaplotrange},...
             'MaxGenerations',200, 'MaxStallGenerations', 20); 

[Result.X, fobj(1), Result.GAexitflag, Result.GAoutput, Result.GApopulation, Result.GAscores] = ga(@(X)obj_fun(X, terr, input_param), length(x0), [], [], [], [], lb, ub,[],intcon, options);

% Plot Winning Robot
figure(2)
best_robot = decodeGenome(Result.X, terr, input_param);
best_robot.pathfind();
best_robot.plotPath();

function [cost] = obj_fun(X, terr, input_param)
    rob = decodeGenome(X, terr, input_param);
    rob.pathfind();
   
    cost = totalCost(rob.sensorCost(), ...
        rob.costOfPath(input_param(5)), input_param(3), input_param(4));
end

function [robot] = decodeGenome(X, terr, input_param)
    i_max_sensors = input_param(6);
    i_range = input_param(7:8);
    i_direction = input_param(9:10);
    i_el_acc = input_param(11:12);
    i_obs_acc = input_param(13:14);

    nS = X(i_max_sensors);
    R = X(i_range(1):i_range(2));
    D = X(i_direction(1):i_direction(2));
    Ael = X(i_el_acc(1):i_el_acc(2));
    Aob = X(i_obs_acc(1):i_obs_acc(2));

    Directions = [1, 2, 3, 4, D];

    for jj  = 1 : nS
        sensor_cost = sensorCost(R(jj), Ael(jj), Aob(jj), input_param(1), input_param(2));
        elevAcc = getAcc(Ael(jj), 5, R(jj));
        obsAcc = getAcc(Aob(jj), 1, R(jj));
        sensors(jj) = RobotSensor(sensor_cost, elevAcc, obsAcc);
    end

    % setting up the sensor configuration
    rig = SensorConfiguration();
    dirs = Directions(1: nS);
    rig.addSensors(sensors, dirs)
    
    %setting up robots
    robot = Robot(rig, terr);
end

function [cost] = totalCost(sensor_cost, path_cost, max_sensor, best_path)
    if path_cost == Inf
        cost = 10; %Cost for not reaching goal
    else 
        cost = (sensor_cost / max_sensor + (1 - best_path / path_cost)) / 2;
    end
end

function [cost] = sensorCost(range, el_acc, obs_acc, c1, c2)
    cost = c1 * range + c2 * el_acc + c2 * obs_acc;
end


function Acc = getAcc(x, per, nRep)
    Acc(1, 1) = x;
    for ii = 2 : nRep
        Acc(1, ii) = max(0, (1 - per/100)  .* Acc(1, ii - 1));
    end
end
