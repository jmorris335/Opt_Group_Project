function [cost] = totalCost(sensor_cost, path_cost, max_sensor, best_path)
    norm_sensor_cost = sensor_cost / max_sensor;
    if path_cost == Inf
        norm_path_cost = 3;
    else
        norm_path_cost = (1 - best_path / path_cost);
    end
    cost = (norm_sensor_cost + norm_path_cost) / 2;
end