classdef SensorConfiguration
   % SensorConfiguration Defines a variety of sensors and standard methods 
   % for configuring them
   
   properties
        sensors         % Cell array containing all the sensors
   end
   methods
       function obj = SensorConfiguration()
       % Basic constructor 
       end
       
       function [] = addSensor(obj, sensor)
       % Adds a sensor to the sensor array
           obj.sensors{end + 1} = sensor;
       end
       
   end
end