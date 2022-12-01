run_case = 2;

switch run_case
    %% Case 1 (Normal):
    case 1
    rng(30);
    N = 20;
    max_height = 3;
    steepness = 0.25;
    density = 0.2;
    terr = Terrain(N, max_height, steepness, density);
    
    %% Case 2 (Steep, and Obstacles)
    case 2
    rng(31);
    N = 20;
    max_height = 3;
    steepness = 0.5;
    density = 0.35;
    terr = Terrain(N, max_height, steepness, density);
end

%% Plot GA
disp(terr.toString());
[Result, input_param, X]= GA(terr);
plotRobot(Result.X, Result.fobj, terr, input_param)


function [] = plotRobot(X, fobj, terr, input_param)
    % Plot Winning Robot
    figure(2)
    best_robot = decodeGenome(X, terr, input_param);
    best_robot.pathfind();
    best_robot.plotPath();
    subtitle(sprintf('Cost: %.4g', fobj));
end