rng(1)
N = 15;
max_height = 3;
steepness = 0.2;
density = 0.3;
terr = Terrain(N, max_height, steepness, density);

cost = 5;
range = 4;
elv_acc = 1 * ones(1, range);
obs_acc = 1 * ones(1, range);
sensor1 = RobotSensor(cost, elv_acc, obs_acc);

rig = SensorConfiguration();
sensors = [sensor1, sensor1, sensor1, sensor1];
rig.addSensors(sensors, [1, 2, 3, 4]);
rob = Robot(rig, terr, true);
rob.pathfind()
rob.plotPath()