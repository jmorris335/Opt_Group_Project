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


%% OPT Var List
% X(1) = nS # number of Sen #UB = 10 LB = 4 int
% X(2:11)= R # list of sensor range #UB =10  LB = 4 int 
% X(12:17) = D # list of sensors dir #UB = 4 LB = 1 int 
% X(18) = Ael # acc for e #UB = 0.95  LB = 0.85
% X(19) = Aob # acc for ob #UB = 0.99 LB = 0.95

x0 = [4,...
      1,2,3,4,5,6,7,8,9,10,...
      1,2,3,4,1,2,...
      0.9,...
      0.98];
      
  



% cost = obj_fun(x0, terr);

%% Implement Genetic Algorithm
lb = [4,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.85,0.95];
ub = [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 4, 4, 4, 4, 4, 4, 0.95, 0.99];
intcon = [1:17];
options = optimoptions( 'ga', 'PopulationSize', 20, 'FunctionTolerance', 0.01, 'PlotFcn', {@gaplotbestf, @gaplotrange},...
             'MaxGenerations',200, 'MaxStallGenerations', 20); 
[Result.X, fobj(1), Result.GAexitflag, Result.GAoutput, Result.GApopulation, Result.GAscores] = ga(@(X)obj_fun(X, terr), 19, [], [], [], [], lb, ub,[],intcon, options);

%% Setting up a number of random robots 

function [cost] = obj_fun(X, terr)
nS = X(1); % number of Sen #UB = 10 LB = 4 int
R = X(2:11) ;% list of sensor range #UB =10  LB = 4 int
D = X(12:17); % list of sensors dir #UB = 4 LB = 1 int
Ael = X(18);  % acc for e #UB = 0.95  LB = 0.85
Aob = X(19);  % acc for ob #UB = 0.99 LB = 0.95
ROB.nSensors = nS;
Directions = [1, 2, 3, 4, D];
for jj  = 1 : ROB.nSensors
    ROB.Sensor(jj).range = R(jj);
    ROB.Sensor(jj).CostSensors = 5 .* R(jj);
    %         ROB.Sensor(jj).elevAcc = getAcc(elevAccMin + (elevAccMax - elevAccMin) .* rand, 5, ROB(ii).Sensor(jj).range);
    ROB.Sensor(jj).elevAcc = getAcc(Ael, 5, ROB.Sensor(jj).range);
    %         ROB.Sensor(jj).obsAcc = getAcc(obsAccMin + (obsAccMax - obsAccMin) .* rand, 1, ROB(ii).Sensor(jj).range);
    ROB.Sensor(jj).obsAcc = getAcc(Aob, 1, ROB.Sensor(jj).range);
    %         ROB.RobotSensor(jj) = RobotSensor(ROB(ii).Sensor(jj).CostSensors, ROB(ii).Sensor(jj).elevAcc, ROB(ii).Sensor(jj).obsAcc);
    ROB.RobotSensor(jj) = RobotSensor(ROB.Sensor(jj).CostSensors, ROB.Sensor(jj).elevAcc, ROB.Sensor(jj).obsAcc);
end
% setting up the sensor configuration
ROB.rig = SensorConfiguration();
ROB.sensors = ROB.RobotSensor;
%     ROB.dirs = [1, 2, 3, 4, d, 1, (ROB.nSensors - 4))];
ROB.dirs = Directions(1: ROB.nSensors);
ROB.rig.addSensors(ROB.sensors, ROB.dirs)

%setting up robots
ROB.rob = Robot(ROB.rig, terr);

%find the path for the robot
ROB.rob.pathfind();

ROB.totalCost =  sum([ROB.Sensor.CostSensors]) + ROB.rob.costOfPath(1);
cost = ROB.totalCost;

%show results
%     figure
%     ROB.rob.plotPath();
end
 
%%

   
%     ROB(ii).Pos.elevAcc = getAcc(elevAccMin + (elevAccMax - elevAccMin) .* rand, 5, ROB(ii).Pos.range);
%     ROB(ii).Pos.obsAcc = getAcc(obsAccMin + (obsAccMax - obsAccMin) .* rand, 1, ROB(ii).Pos.range);
    




% function Sensor  = makeSensor
% end
% for ii = 1 : nSensors
% sensor1 = RobotSensor(cost, elv_acc, obs_acc);
% 
% % Setup SensorConfiguration
% rig = SensorConfiguration();
% sensors = [sensor1, sensor1, sensor1, sensor1];
% dirs = [1, 2, 3, 4];
% rig.addSensors(sensors, dirs);
% end

function Acc = getAcc(x, per, nRep)
Acc(1, 1) = x;
for ii = 2 : nRep
    Acc(1, ii) = (1 - per/100)  .* Acc(1, ii - 1);
end
end

