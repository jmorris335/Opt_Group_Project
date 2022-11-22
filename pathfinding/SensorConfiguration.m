classdef SensorConfiguration < handle
   % SensorConfiguration Defines a variety of sensors and standard methods 
   % for configuring them
   
   properties
        % sensors - 4x1 Cell array of cell arrays, where each row is the
        % sensors in the SensorConfiguration facing a certain direciton.
        % The order of the directions is specified by the DIRECTION
        % enumeration.
        sensors cell  

        total_cost double {mustBeNonnegative}  % Sum of all sensor costs;

        % elevation_range - 1x4 int array of cells' elevation visible in 4 cardinal directions
        %   Order is prescribed by DIRECTION enumeration
        elevation_range {mustBeInteger} 

        % obstacle_range - 1x4 int array of cells' obstacles visible in 4 cardinal directions
        %   Order is prescribed by DIRECTION enumeration
        obstacle_range {mustBeInteger} 

        % elevation_accuracy - Array of floats for accuracy of elevation detection of each cell
        %   Each row corresponds to a direction with the order specified by
        %   DIRECTION enumeration. Each column correspondes to a cell,
        %   starting with the cell closest to the robot. The number of
        %   columns must be equivalent to the maximum number in elevation_range.
        %
        %   1 is perfect accuracy, 0 is completely random
        elevation_accuracy double {mustBeNonnegative}

        % obstacle_accuracy - Array of floats for accuracies of obstacle detection each cell
        %   Each row corresponds to a direction with the order specified by
        %   DIRECTION enumeration. Each column correspondes to a cell,
        %   starting with the cell closest to the robot. The number of
        %   columns must be equivalent to the maximum number in obstacle_range.
        %
        %   1 is perfect accuracy, 0 is completely random
        obstacle_accuracy double {mustBeNonnegative}
   end

   methods
       function obj = SensorConfiguration(varargin)
       % Basic constructor
            obj.sensors = {{}; {}; {}; {}};
            obj.total_cost = 0;
            obj.elevation_range = zeros(1, 4);
            obj.obstacle_range = zeros(1, 4);
            obj.elevation_accuracy = zeros(4, 1);
            obj.obstacle_accuracy = zeros(4, 1);

            for i = 1:nargin
                if ~isa(varargin(i), 'RobotSensor')
                    return
                end
                addSensor(varargin(i), DIRECTION.RIGHT)
            end
       end

       function [] = addSensors(obj, in_sensors, directions)
       % Bulk sensor addition
            if length(in_sensors) ~= length(directions)
                return
            end

            for i = 1:length(in_sensors)
                obj.addSensor(in_sensors(i), directions(i));
            end
       end
       
       function [] = addSensor(obj, sensor, direction)
       % Adds a sensor to the sensor array
       %    sensor must be RobotSensor object, direction must be a
       %    DIRECTION enumeration

            if ~isa(sensor, 'RobotSensor')
                return
            end

            obj.sensors{direction}{end + 1} = sensor;
            obj.total_cost = obj.total_cost + sensor.cost;
            obj.addRange(sensor, direction);
            obj.addAccuracy(sensor, direction);
       end

       function [] = addRange(obj, sensor, direction)
       % Configures the range of the SensorConfiguration to include any
       %    increased range resulting from the addition of the RobotSensor.
       %
       %    Note that direction must be a DIRECTION object and sensor must be
       %    a RobotSensor object

            if ~isa(sensor, 'RobotSensor')
                return
            end
            
            diff = sensor.elevation_range - obj.elevation_range(direction);
            if diff > 0
                obj.elevation_range(direction) = sensor.elevation_range;
                obj.elevation_accuracy = [obj.elevation_accuracy, zeros(4, diff-1)];
            end

            diff = sensor.obstacle_range - obj.obstacle_range(direction);
            if diff > 0
                obj.obstacle_range(direction) = sensor.obstacle_range;
                obj.obstacle_accuracy = [obj.obstacle_accuracy, zeros(4, diff-1)];
            end
       end

       function [] = addAccuracy(obj, sensor, direction) 
       % Configures the accuracy parameters to include the accuracy of a
       %    new sensor in the given direction. The method used for doing this
       %    is given by the function handle 'new'. This method must be called
       %    after addRange.
       %
       %    Note that direction must be a DIRECTION object and sensor must be
       %    a RobotSensor object

            if ~isa(sensor, 'RobotSensor')
                return
            end

            new = @(this, that) this + (1 - this) * that;

            for i = 1:sensor.elevation_range
                this = obj.elevation_accuracy(direction, i);
                that = sensor.elevation_accuracy(i);
                   
                obj.elevation_accuracy(direction, i) = min(1, new(this, that));
            end

            for i = 1:sensor.obstacle_range
                this = obj.obstacle_accuracy(direction, i);
                that = sensor.obstacle_accuracy(i);
                    
                obj.obstacle_accuracy(direction, i) = min(1, new(this, that));
            end
       end

       function [out] = getSensors(obj, direction) 
       % Returns all the sensors for a given direction. 
       %
       % Note that direction must be a DIRECTION enumeration.

            if ~isa(direction, 'DIRECTION')
                out = obj.sensors;
                return
            end
            out = obj.sensors{direction};
       end
   end
end









