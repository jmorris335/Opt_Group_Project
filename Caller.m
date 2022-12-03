clear

% Declare random seed
rng(30);

% Create Sensors
cost = 5;
elv_acc = [1, 1, 1, 1];
obs_acc = [1, 1, 1, 1];
sensor1 = RobotSensor(cost, elv_acc, obs_acc);

% Setup SensorConfiguration
rig = SensorConfiguration();
sensors = [sensor1, sensor1, sensor1, sensor1, sensor1];
dirs = [1, 2, 3, 4, 2];
rig.addSensors(sensors, dirs);

% Setup terrain
N = 30;
max_height = 3;
steepness = 0.25;
density = 0.2;
terr = Terrain(N, max_height, steepness, density);
disp(terr.toString());

% Setup robot
rob = Robot(rig, terr);
disp(rob.toString());

% Find path
rob.pathfind();

% Show results
disp(rob.toString());
rob.plotPath();
