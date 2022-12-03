function [cost] = totalCost(sensor_cost, path_cost, max_sensor, best_path)
% Returns the normalized cost of the robot based on the sensor
% configuration and length of the path. 
% 
% Best possible is 0: ideal path and zero-accuracy sensors
% 1 represents a worst-case scenario robot that reached the goal
% >1 represents a robot that did not reach the goal
% Worst possible is 1.5: a maximum accuracy sensors that didn't reach the goal

    norm_sensor_cost = sensor_cost / max_sensor;
    if path_cost == Inf
        norm_path_cost = 2;
    else
        norm_path_cost = (1 - best_path / path_cost);
    end
    cost = (norm_sensor_cost + norm_path_cost) / 2;
end