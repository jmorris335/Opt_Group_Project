function [robot] = decodeGenome(X, terr, input_param)
    i_max_sensors = input_param(6);
    i_range = input_param(7:8);
    i_direction = input_param(9:10);
    i_el_acc = input_param(11:12);
    i_obs_acc = input_param(13:14);

    nS = X(i_max_sensors);
    R = X(i_range(1):i_range(2));
    D = X(i_direction(1):i_direction(2));
    Ael = X(i_el_acc(1):i_el_acc(2));
    Aob = X(i_obs_acc(1):i_obs_acc(2));

    Directions = [1, 2, 3, 4, D];

    for jj  = 1 : nS
        sensor_cost = sensorCost(R(jj), Ael(jj), Aob(jj), input_param(1), input_param(2));
        elevAcc = getAcc(Ael(jj), 5, R(jj));
        obsAcc = getAcc(Aob(jj), 1, R(jj));
        sensors(jj) = RobotSensor(sensor_cost, elevAcc, obsAcc);
    end

    % setting up the sensor configuration
    rig = SensorConfiguration();
    dirs = Directions(1: nS);
    rig.addSensors(sensors, dirs)
    
    %setting up robots
    robot = Robot(rig, terr);
end

function [cost] = sensorCost(range, el_acc, obs_acc, c1, c2)
    cost = c1 * range + c2 * el_acc + c2 * obs_acc;
end


function Acc = getAcc(x, per, nRep)
    Acc(1, 1) = x;
    for ii = 2 : nRep
        Acc(1, ii) = max(0, (1 - per/100)  .* Acc(1, ii - 1));
    end
end