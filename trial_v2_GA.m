clc;
nP = 20; % number of particles
nSenMin = 4;
nSenMax = 10;
rangeMin = 4;
rangeMax = 10;

elevAccMin = 0.85;
elevAccMax = 0.95;
obsAccMin = 0.95;
obsAccMax = 0.99;

% ROB = repmat([], nP, 1);

%% Now let us set up the terrain
N = 20;
max_height = 3;
steepness = 0.25;
density = 0.2;
terr = Terrain(N, max_height, steepness, density);
disp(terr.toString());

%% FIXME
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

max_sensor_cost = sensorCost(max_range, max_el_acc, max_obs_acc, cost_range, cost_acc);
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

function [cost] = obj_fun(X, terr, input_param)
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

    ROB.nSensors = nS;
    Directions = [1, 2, 3, 4, D];

    for jj  = 1 : ROB.nSensors
        ROB.Sensor(jj).range = R(jj);
        ROB.Sensor(jj).CostSensors = sensorCost(R(jj), Ael(jj), Aob(jj), input_param(1), input_param(2));
        ROB.Sensor(jj).elevAcc = getAcc(Ael(jj), 5, ROB.Sensor(jj).range);
        ROB.Sensor(jj).obsAcc = getAcc(Aob(jj), 1, ROB.Sensor(jj).range);
        ROB.RobotSensor(jj) = RobotSensor(ROB.Sensor(jj).CostSensors, ROB.Sensor(jj).elevAcc, ROB.Sensor(jj).obsAcc);
    end

    % setting up the sensor configuration
    ROB.rig = SensorConfiguration();
    ROB.sensors = ROB.RobotSensor;
    ROB.dirs = Directions(1: ROB.nSensors);
    ROB.rig.addSensors(ROB.sensors, ROB.dirs)
    
    %setting up robots
    ROB.rob = Robot(ROB.rig, terr);
    
    %find the path for the robot
    ROB.rob.pathfind();
   
    cost = totalCost(ROB.rig.total_cost / ROB.nSensors, ...
        ROB.rob.costOfPath(input_param(5)), input_param(3), input_param(4));
end

function [cost] = sensorCost(range, el_acc, obs_acc, c1, c2)
    cost = c1 * range + c2 * el_acc + c2 * obs_acc;
end

function [cost] = totalCost(sensor_cost, path_cost, max_sensor, best_path)
    cost = (sensor_cost / max_sensor + (1 - best_path / path_cost)) / 2;
end

%END FIXME


% %% OPT Var List
% % X(1) = nS # number of Sen #UB = 10 LB = 4 int
% % X(2:11)= R # list of sensor range #UB =10  LB = 4 int 
% % X(12:17) = D # list of sensors dir #UB = 4 LB = 1 int 
% % X(18) = Ael # acc for e #UB = 0.95  LB = 0.85
% % X(19) = Aob # acc for ob #UB = 0.99 LB = 0.95
% 
% % x0 = [4,...
% %       1,2,3,4,5,6,7,8,9,10,...
% %       1,2,3,4,1,2,...
% %       0.9,...
% %       0.98];
% 
% % cost = obj_fun(x0, terr);
% 
% %% Implement Genetic Algorithm
% % lb = [4,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.85,0.95];
% % ub = [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 4, 4, 4, 4, 4, 4, 0.95, 0.99];
% % intcon = [1:17];
% % options = optimoptions( 'ga', 'PopulationSize', 20, 'FunctionTolerance', 0.01, 'PlotFcn', {@gaplotbestf, @gaplotrange},...
% %              'MaxGenerations',200, 'MaxStallGenerations', 20); 
% % [Result.X, fobj(1), Result.GAexitflag, Result.GAoutput, Result.GApopulation, Result.GAscores] = ga(@(X)obj_fun(X, terr), 19, [], [], [], [], lb, ub,[],intcon, options);
% 
% %% Setting up a number of random robots 
% 
% function [cost] = obj_fun(X, terr)
% nS = X(1); % number of Sen #UB = 10 LB = 4 int
% R = X(2:11) ;% list of sensor range #UB =10  LB = 4 int
% D = X(12:17); % list of sensors dir #UB = 4 LB = 1 int
% Ael = X(18);  % acc for e #UB = 0.95  LB = 0.85
% Aob = X(19);  % acc for ob #UB = 0.99 LB = 0.95
% ROB.nSensors = nS;
% Directions = [1, 2, 3, 4, D];
% for jj  = 1 : ROB.nSensors
%     ROB.Sensor(jj).range = R(jj);
%     ROB.Sensor(jj).CostSensors = 5 .* R(jj);
%     %         ROB.Sensor(jj).elevAcc = getAcc(elevAccMin + (elevAccMax - elevAccMin) .* rand, 5, ROB(ii).Sensor(jj).range);
%     ROB.Sensor(jj).elevAcc = getAcc(Ael, 5, ROB.Sensor(jj).range);
%     %         ROB.Sensor(jj).obsAcc = getAcc(obsAccMin + (obsAccMax - obsAccMin) .* rand, 1, ROB(ii).Sensor(jj).range);
%     ROB.Sensor(jj).obsAcc = getAcc(Aob, 1, ROB.Sensor(jj).range);
%     %         ROB.RobotSensor(jj) = RobotSensor(ROB(ii).Sensor(jj).CostSensors, ROB(ii).Sensor(jj).elevAcc, ROB(ii).Sensor(jj).obsAcc);
%     ROB.RobotSensor(jj) = RobotSensor(ROB.Sensor(jj).CostSensors, ROB.Sensor(jj).elevAcc, ROB.Sensor(jj).obsAcc);
% end
% % setting up the sensor configuration
% ROB.rig = SensorConfiguration();
% ROB.sensors = ROB.RobotSensor;
% %     ROB.dirs = [1, 2, 3, 4, d, 1, (ROB.nSensors - 4))];
% ROB.dirs = Directions(1: ROB.nSensors);
% ROB.rig.addSensors(ROB.sensors, ROB.dirs)
% 
% %setting up robots
% ROB.rob = Robot(ROB.rig, terr);
% 
% %find the path for the robot
% ROB.rob.pathfind();
% 
% ROB.totalCost =  sum([ROB.Sensor.CostSensors]) + ROB.rob.costOfPath(1);
% cost = ROB.totalCost;
% 
% %show results
% %     figure
% %     ROB.rob.plotPath();
% end
%  
% %%
% 

function Acc = getAcc(x, per, nRep)
    Acc(1, 1) = x;
    for ii = 2 : nRep
        Acc(1, ii) = max(0, (1 - per/100)  .* Acc(1, ii - 1));
    end
end

