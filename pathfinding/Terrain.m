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
        p_rand PsuedoRandom % For deterministic pathfinding
        color_map {mustBeNonnegative} % For plotting elevations

        % Maps
        
        elevation_map   % nxn array of the elevation of the terrain
        obstacle_map    % nxn array of booleans, 0 is pass, 1 is no pass

        % Environment variables
        
        PEAK_HEIGHT double {mustBePositive} % maximum elevation on map
        n {mustBeInteger, mustBePositive}   % Amount of cells per side
    end
    
    methods
        %% Construction
        function obj = Terrain(cells_per_side, peak_height, steepness, density)
        % Basic constructor for class
            obj.n = cells_per_side;
            obj.PEAK_HEIGHT = peak_height;
            obj.elevation_map = obj.createTopography(steepness);
            obj.obstacle_map = obj.createObstacles(density);
            obj.p_rand = PsuedoRandom();
            obj.makeColorMap();
        end
        
        function [topo_out] = createTopography(obj, steepness)
        % Returns a randomized map of the topography
        %   steepness is a normalized parameter for changing the randomness 
        %   (elevation change) of the topography. 0 is perfectly flat, 1 is 
        %   random cliffs
            
            diff = steepness * obj.PEAK_HEIGHT;

            topo_out = NaN(obj.n);
            topo_out(1, 1) = 0;

            surrounding = @(i, n) unique([max(1,i-n-1):max(1,i-n+1), max(1,i-1), ...
                min(n,i+1), min(n^2,i+n-1):min(n^2,i+n+1)]);

            %Create cluster
            for i = 1:numel(topo_out)
                avg = mean([topo_out(surrounding(i, obj.n))], 'omitnan');
                rnd = 2*diff * rand - diff;
                topo_out(i) = min(obj.PEAK_HEIGHT, max(0, avg + rnd));
            end
        end
        
        function [obstacles_out] = createObstacles(obj, density)
        % Returns a randomized map of the obstacles in the terrain

            obstacles_out = rand(obj.n, obj.n) < density;
            obstacles_out(1, 1) = 0;
            obstacles_out(obj.n, obj.n) = 0;
        end

        function [color_map] = makeColorMap(obj)
        % Creates the color map for plotting the elevations.
        %   Yellow is high, green is low
        obj.color_map = [0.5, 0.8, 0.5;...
                         0.6, 0.9, 0.5;...
                         0.8, 1.0, 0.5;...
                         0.9, 1.0, 0.5;...
                         1.0, 1.0, 0.7;...
                         0.0, 0.0, 0.0];
        color_map = obj.color_map;
        end

        function [index_map] = getColorMapIndex(obj)
        % Returns an index to the color the elevation should map to.
        index_map = zeros(size(obj.elevation_map));
        ratios = obj.elevation_map ./ obj.PEAK_HEIGHT;
        h = height(obj.color_map);
        vals = 0:1/(h-1):(h-2)/(h-1);
            for i = 1:height(obj.elevation_map)
                for j = 1:width(obj.elevation_map)
                    [~, index_map(i, j)] = min(abs(ratios(i, j) - vals));
                    if obj.getObstacleAt(i, j)
                        index_map(i, j) = h;
                    end
                end
            end
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
        function [] = reset(obj, new_set)
        % Resets the PsuedoRandom property. If called before pathfinding
        % allows for randomized, yet deterministic pathfinding simulations.
            obj.p_rand.set(new_set);
        end

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
                error = obj.PEAK_HEIGHT * (-1 + 2* obj.p_rand.getFloat() ) * (1 - accuracy);
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
                error = obj.p_rand.getFloat() > accuracy;
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
                    out = append(out, sprintf('%.2f ', elvs(i, j)));
                end
                out = append(out, newline);
            end

            out = append(out, sprintf('\n\tObstacles: \n'));
            obss = string(uint8(obj.obstacle_map));
            for i = 1:obj.n
                out = append(out, sprintf('\t'));
                for j = 1:obj.n
                    out = append(out, sprintf(obss(i, j) + sprintf(' ')));
                end
                out = append(out, newline);
            end
        end

        function [p] = plot(obj)
        % Plots the terrain and returns a handle to the plot
            x = (0:obj.n) + 0.5;
            y = (0:obj.n) + 0.5;
            [x, y] = meshgrid(x, y);
            c = obj.getColorMapIndex();
            c_map = obj.color_map;
            k = height(c_map);
            c = [c, k*ones(height(c), 1); k*ones(1, width(c)+1)];
            p = pcolor(x, obj.n-y+1, c);
            colormap(c_map)
        end
    end
end