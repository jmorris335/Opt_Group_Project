for run_case = 1

    switch run_case
        case 1 % Normal
        rng(30);
        N = 20;
        max_height = 3;
        steepness = 0.25;
        density = 0.2;
    
        case 2 % Increased obstacles
        rng(21);
        N = 20;
        max_height = 3;
        steepness = 0.25;
        density = 0.35;
    
        case 3 % Increased steepness
        rng(31);
        N = 20;
        max_height = 5;
        steepness = 0.5;
        density = 0.2;
    
        case 4 % Normal, but increased N
        rng(40);
        N = 40;
        max_height = 3;
        steepness = 0.25;
        density = 0.2;
        
        case 5 % Bad case, everything high
        rng(31);
        N = 40;
        max_height = 5;
        steepness = 0.5;
        density = 0.35;
    end

    terr{run_case} = Terrain(N, max_height, steepness, density);
%     disp(terr.toString());
    [Result{run_case}, input_param{run_case}]= GA(terr{run_case});
    best_robot{run_case} = decodeGenome(Result{run_case}.X, terr{run_case}, input_param{run_case});
%     plotRobot(best_robot, Result.Fval)

end

%% Plot GA
% terr = Terrain(N, max_height, steepness, density);
% % disp(terr.toString());
% [Result, input_param]= GA(terr);
% best_robot = decodeGenome(Result.X, terr, input_param);
% plotRobot(best_robot, Result.Fval)

function [] = plotRobot(best_robot, Fval)
    % Plot Winning Robot
    figure(2)
    best_robot.pathfind();
    best_robot.plotPath();
    subtitle(sprintf('Cost: %.4g', Fval));
end