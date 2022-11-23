# Pathfinding Overview
## Contents:
1. Robot.m : Robot traverses the terrain via a given SensorConfiguration
2. SensorConfiguration.m : A collection of Sensors and their aggregate properities
3. Sensor.m : A given collection of ranges and accuracies
4. Terrain.m : The environment the robot navigates
5. DIRECTION.m : An enumeration of the types of moves (that sets the order moves are checked in)

## Example
```matlab
% Declare random seed
rng(31);

% Create Sensors
elv_acc = [1, 1, 1, 1];
obs_acc = [1, 1, 1, 1];
sensor1 = RobotSensor(5, elv_acc, obs_acc);

% Setup SensorConfiguration
rig = SensorConfiguration();
sensors = [sensor1, sensor1, sensor1, sensor1];
dirs = [1, 2, 3, 4];
rig.addSensors(sensors, dirs);

% Setup terrain
N = 5;
max_height = 3;
steepness = 0.25;
density = 0.3;
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
```
