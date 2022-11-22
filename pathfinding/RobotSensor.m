classdef RobotSensor
    % RobotSensor a template for a sensor that goes on the robot. The master
    % list for all sensor parameters. A robot has a SensorConfiguration,
    % which in turn has a collection of Sensors.
    
    properties
        % visibility - 1x2 array, pixels visible to sensor
        visibility

        % obstacle_detection - normalized value for detecting obstacles
        %   1 is perfect reliability, 0 is never accurate
        obstacle_detection
        
        % lighting_reliance - normalized value for sensitivity to light
        %   1 is needs full light, 0 is can operate in complete darkness
        lighting_reliance

        % cost - the cost of the individual RobotSensor
        cost
    end
    
    methods
        function obj = RobotSensor(cost)
        % Basic constructor
        obj.cost = cost;
        end
    end    
end