classdef Terrain < handle
    % Terrain Details of terrain for robotic path-finding
    %   This class provides the full definition of the environment that a
    %   simulated robot will traverse. Its parameters include several maps 
    %   of various terrain features (such as elevation, obstacles, etc.)
    %
    %   Robots traverse the terrain by utilizing a set of access functions 
    %   that return an estimate of the maps based on the parameters of the
    %   robot's sensor configuration.
    % 
    %   The terrain is detailed as set of nxn 2D maps as well as a set of
    %   environmental variables. Note that the robot always starts at (1,1) 
    %   and ends at (n,n)
    
    properties    
        % Maps
        
        elevation_map   % nxn array of the elevation of the terrain
        obstacle_map    % nxn array of booleans, 0 is pass, 1 is no pass

        % Environment variables
        
        PEAK_HEIGHT double {mustBePositive} % maximum elevation on map
        n {mustBeInteger, mustBePositive}   % Amount of cells per side
    end
    
    methods
        %% Construction
        function obj = Terrain(cells_per_side, peak_height)
        % Basic constructor for class
            obj.n = cells_per_side;
            obj.PEAK_HEIGHT = peak_height;
            obj.elevation_map = obj.createTopography();
            obj.obstacle_map = obj.createObstacles();
        end
        
        function [topography_out] = createTopography(obj)
        % Returns a randomized map of the topography
            topography_out = randi(obj.PEAK_HEIGHT, obj.n);
        end
        
        function [obstacles_out] = createObstacles(obj)
        % Returns a randomized map of the obstacles in the terrain
            obstacles_out = randi(2, obj.n) - 1;
            obstacles_out(1, 1) = 0;
            obstacles_out(obj.n, obj.n) = 0;
        end

        %% Access
        function [out] = getElevationAt(obj, row, col)
        % Getter for elevation_map
            out = obj.elevation_map(row, col);
        end

        function [out] = getObstacleAt(obj, row, col)
        % Getter for obstacle_map
            out = obj.obstacle_map(row, col);
        end

        %% Sensor Functions
        function [out_elvs] = senseElevations(obj, row, col, direction, accuracy)
        % Returns an array of elevations based on the accuracy of inputted 
        %   sensor in the given direction
            out_elvs = zeros(1, length(accuracy));
            for i = 1:length(accuracy)
                [row, col] = direction.move1cell(row, col);
                out_elvs(i) = obj.senseElevationAt(row, col, accuracy(i));
            end
        end

        function [out_elv] = senseElevationAt(obj, row, col, accuracy)
        % Returns the elevation at the inputted row and column adjusted at
        %   the inputted accuracy
            if (row > obj.n) || (col > obj.n) || (row <= 0) || (col <= 0) || accuracy == 0
                    out_elv = NaN;
            else
                real = obj.elevation_map(row, col);
                error = obj.PEAK_HEIGHT * (-1 + 2*rand(1)) * (1 - accuracy);
                out_elv = max(min(real + error, obj.PEAK_HEIGHT), 0);
            end
        end

        function [out_obss] = senseObstacles(obj, row, col, direction, accuracy)
        % Returns an array of go/no-go booleans based on the accuracy of inputted 
        %   sensor in the given direction
            out_obss = zeros(1, length(accuracy));
            for i = 1:length(accuracy)
                [row, col] = direction.move1cell(row, col);
                out_obss(i) = obj.senseObstacleAt(row, col, accuracy(i));
            end
        end

        function [out_obs] = senseObstacleAt(obj, row, col, accuracy)
        % Returns whether the sensor detected an obstacle at the inputted row 
        %   and column based on the inputted accuracy
            if (row > obj.n) || (col > obj.n) || (row <= 0) || (col <= 0) || accuracy == 0
                    out_obs = NaN;
            else
                error = rand(1) > accuracy;
                if error
                    out_obs = ~obj.obstacle_map(row, col);
                else
                    out_obs = obj.obstacle_map(row, col);
                end
            end
        end

        %% Display
        function [out] = toString(obj)
            out = [sprintf('Terrain Class\n'), ...
            sprintf('\tElevation: \n')];

            elvs = string(obj.elevation_map);
            for i = 1:obj.n
                out = append(out, sprintf('\t'));
                for j = 1:obj.n
                    out = append(out, sprintf(elvs(i, j) + sprintf(' ')));
                end
                out = append(out, newline);
            end

            out = append(out, sprintf('\n\tObstacles: \n'));
            obss = string(obj.obstacle_map);
            for i = 1:obj.n
                out = append(out, sprintf('\t'));
                for j = 1:obj.n
                    out = append(out, sprintf(obss(i, j) + sprintf(' ')));
                end
                out = append(out, newline);
            end
        end
    end
end