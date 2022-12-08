clear
load('results_variable2.mat')

% Recalculate Fvals
for i = 1:5
    br = best_robot{i};
    br.pathfind()
    N(i, 1) = terr{i}.n;
    Max_Height(i, 1) = terr{i}.PEAK_HEIGHT;
    Sensor_Max_Cost(i, 1) = input_param{i}(3);
    Sensor_Cost(i, 1) = br.sensorCost;
    Path_Max_Cost(i, 1) = input_param{i}(4);
    Path_Cost(i, 1) = br.costOfPath(1);
    FinalCost(i, 1) = totalCost(Sensor_Cost(i), Path_Cost(i), ...
        Sensor_Max_Cost(i), Path_Max_Cost(i));
    El_Range_Up(i, 1) = br.rig.elevation_range(1);
    El_Range_Down(i, 1) = br.rig.elevation_range(2);
    El_Range_Left(i, 1) = br.rig.elevation_range(3);
    El_Range_Right(i, 1) = br.rig.elevation_range(4);
    Mean_El_Acc_Up(i, 1) = mean(nonzeros(br.rig.elevation_accuracy(1,:)));
    Mean_El_Acc_Down(i, 1) = mean(nonzeros(br.rig.elevation_accuracy(2,:)));
    Mean_El_Acc_Left(i, 1) = mean(nonzeros(br.rig.elevation_accuracy(3,:)));
    Mean_El_Acc_Right(i, 1) = mean(nonzeros(br.rig.elevation_accuracy(4,:)));
    Obs_Range_Up(i, 1) = br.rig.obstacle_range(1);
    Obs_Range_Down(i, 1) = br.rig.obstacle_range(2);
    Obs_Range_Left(i, 1) = br.rig.obstacle_range(3);
    Obs_Range_Right(i, 1) = br.rig.obstacle_range(4);
    Mean_Obs_Acc_Up(i, 1) = mean(nonzeros(br.rig.obstacle_accuracy(1,:)));
    Mean_Obs_Acc_Down(i, 1) = mean(nonzeros(br.rig.obstacle_accuracy(2,:)));
    Mean_Obs_Acc_Left(i, 1) = mean(nonzeros(br.rig.obstacle_accuracy(3,:)));
    Mean_Obs_Acc_Right(i, 1) = mean(nonzeros(br.rig.obstacle_accuracy(4,:)));
end

results = table(Sensor_Max_Cost, Sensor_Cost, Path_Max_Cost, Path_Cost, FinalCost, ...
    El_Range_Up, El_Range_Down, El_Range_Left, El_Range_Right,...
    Mean_El_Acc_Up, Mean_El_Acc_Down, Mean_El_Acc_Left, Mean_El_Acc_Right,...
    Obs_Range_Up, Obs_Range_Down, Obs_Range_Left, Obs_Range_Right,...
    Mean_Obs_Acc_Up, Mean_Obs_Acc_Down, Mean_Obs_Acc_Left, Mean_Obs_Acc_Right);
writetable(results, 'Results Table.xlsx');
