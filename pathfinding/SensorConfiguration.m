classdef SensorConfiguration < handle
   % SensorConfiguration Defines a variety of sensors and standard methods 
   % for configuring them
   
   properties
        sensors                         % Cell array of all sensors
        total_cost {mustBePositive}     % Sum of all sensor costs;

        range_UP {mustBeInteger}        % Number of cells visible UP
        range_DWN {mustBeInteger}       % Number of cells visible DOWN
        range_LFT {mustBeInteger}       % Number of cells visible LEFT
        range_RHT {mustBeInteger}       % Number of cells visible RIGHT

        % accuracy_UP - Array of floats for accuracies of each cell UP
        %   Must be length of range_UP
        %   1 is perfect accuracy, 0 is completely random
        accuracy_UP {mustBePositive}

        % accuracy_DWN - Array of floats for accuracies of each cell DOWN
        %   Must be length of range_UP
        %   1 is perfect accuracy, 0 is completely random
        accuracy_DWN {mustBePositive}

        % accuracy_LFT - Array of floats for accuracies of each cell LEFT
        %   Must be length of range_UP
        %   1 is perfect accuracy, 0 is completely random
        accuracy_LFT {mustBePositive}

        % accuracy_RHT - Array of floats for accuracies of each cell RIGHT
        %   Must be length of range_UP
        %   1 is perfect accuracy, 0 is completely random
        accuracy_RHT {mustBePositive}
   end

   methods
       function obj = SensorConfiguration()
       % Basic constructor
            obj.sensors = {};
            obj.total_cost = 0;
       end
       
       function [] = addSensor(obj, sensor)
       % Adds a sensor to the sensor array
       %    sensor must be RobotSensor object

            if isa(sensor, 'RobotSensor') == 0
                return
            end

            obj.sensors{end + 1} = sensor;
            obj.total_cost = obj.total_cost + sensor.cost;
       end

       function [] = addRange(obj, sensor, direction)
       
   end
end