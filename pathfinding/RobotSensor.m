classdef RobotSensor
    % RobotSensor a template for a sensor that goes on the robot. The master
    % list for all sensor parameters. A robot has a SensorConfiguration,
    % which in turn has a collection of Sensors.
    
    properties
        % elevation_range - The number of cells for which the sensor can detect the elevation
        elevation_range {mustBeInteger}

        % obstacle_range - The number of cells for which the sensor can detect obstacles
        obstacle_range {mustBeInteger}

        % elevation_accuracy - 1xn array of the accuracy the sensor can
        %   detect the elevation for each cell, where n (the length of the
        %   array) is the elevation range.
        %   1 is perfect accuracy, 0 is completely random.
        elevation_accuracy double {mustBeNonnegative}

        % obstacle_accuracy - 1xn array of the accuracy the sensor can
        %   detect obstacles in each cell, where n (the length of the
        %   array) is the elevation range.
        %   1 is perfect accuracy, 0 is completely random.
        obstacle_accuracy double {mustBeNonnegative}        

        % cost - the cost of the individual RobotSensor
        cost double {mustBeNonnegative}
    end
    
    methods
        function obj = RobotSensor(cost, elv_range, obs_range, elv_acc, obs_acc)
        % Basic constructor
        obj.cost = cost;
        obj.elevation_range = elv_range;
        obj.obstacle_range = obs_range;

        temp = zeros(1, elv_range);
        temp(1:min(length(elv_acc), elv_range)) = elv_acc(1:min(length(elv_acc), elv_range));
        obj.elevation_accuracy = temp;

        temp = zeros(1, obs_range);
        temp(1:min(length(obs_acc), obs_range)) = obs_acc(1:min(length(obs_acc), obs_range));
        obj.obstacle_accuracy = temp;
        end
    end    
end