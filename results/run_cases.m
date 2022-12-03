run_case = 1;

switch run_case
    case 1 % Normal
    rng(30);
    N = 20;
    max_height = 3;
    steepness = 0.25;
    density = 0.2;
    terr = Terrain(N, max_height, steepness, density);
    
    case 2 % Steep, and Obstacles
    rng(31);
    N = 20;
    max_height = 3;
    steepness = 0.5;
    density = 0.35;
    terr = Terrain(N, max_height, steepness, density);
end

%% Plot GA
% disp(terr.toString());
[Result, input_param]= GA(terr);
best_robot = decodeGenome(Result.X, terr, input_param);
plotRobot(best_robot, Result.Fval)

function [] = plotRobot(best_robot, Fval)
    % Plot Winning Robot
    figure(2)
    best_robot.pathfind();
    best_robot.plotPath();
    subtitle(sprintf('Cost: %.4g', Fval));
end