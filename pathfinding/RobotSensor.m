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

        function out = toString(obj)
            out = append(sprintf('Sensor'), newline);
            out = append(out, sprintf('\tElevation Range: %d', obj.elevation_range));
            out = append(out, newline, sprintf('\tObstacle Range: %d', obj.obstacle_range));
            out = append(out, newline, sprintf('\tElevation Accuracy: ['));
            for i = 1:length(obj.elevation_accuracy)
                out = append(out, sprintf('%.2g, ', obj.elevation_accuracy(i)));
            end
            out = append(out, ']');
            out = append(out, newline, sprintf('\tObstacle Accuracy: ['));
            for i = 1:length(obj.obstacle_accuracy)
                out = append(out, sprintf('%.2g, ', obj.obstacle_accuracy(i)));
            end
            out = append(out, ']');
        end
    end    
end