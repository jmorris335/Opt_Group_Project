classdef Robot < handle
    % Robot Navigates a Terrain object.
    %   The Robot contains a SensorConfiguration which it uses to perceive
    %   the Terrain maps. It uses repeated calls to the Terrain, passing
    %   its SensorConfiguration to build up a internal knowledge of the
    %   map. It then traverses the map via an A* pathfinding algorithm.

    properties (Access=private)
        rig SensorConfiguration
        terrain Terrain
        position {mustBeInteger, mustBeNonnegative}

        % known_nodes - an nxn array of values about the state of each node
        %   as known to the robot. NaN indicates the node has not been
        %   explored. 0 indicates the node has been explored but not
        %   visited. 1 indicates the node has been visited.
        known_nodes

        elevation_map double
        obstacle_map
    end
    
    methods
        %% Construction
        function obj = Robot(rig, terrain)
        % Basic constructors
            obj.rig = rig;
            obj.position = [1, 1];
            obj.terrain = terrain;
            obj.elevation_map = NaN(terrain.n);
            obj.obstacle_map = NaN(terrain.n);
            obj.known_nodes = NaN(terrain.n);
            obj.elevation_map(1, 1) = terrain.getElevationAt(1, 1);
            obj.obstacle_map(1, 1) = terrain.getObstacleAt(1, 1);
            obj.known_nodes(1, 1) = 1;
        end

        %% Access
        function out = row(obj)
        % Return the row component of the current position
            out = obj.position(1);
        end

        function out = col(obj)
        % Return the column component of the current position
            out = obj.position(2);
        end

        function out = totalSteps(obj)
        % returns total steps taken by the robot up to this point
            out = sum((obj.known_nodes == 1), 'all');
        end

        function [] = placeAt(obj, row, col)
        % Places the robot on an arbitrary node in the grid
            row = min(max(row, 0), obj.terrain.n);
            col = min(max(col, 0), obj.terrain.n);
            obj.position = [row, col];
        end

        %% Navigation
        function [] = exploreHere(obj)
        % Explores nodes connected to the current position within sensor range
            obj.senseElevation();
            obj.senseObstacles();
        end

        function [] = senseElevation(obj)
        % Calls the terrain methods to sense the elevation about the current position
            for i = 1:4
                dir = DIRECTION(i);
                elvs = obj.terrain.senseElevations(obj.row, obj.col, dir, ...
                    obj.rig.elevation_accuracy(i, :));
                r = obj.row;
                c = obj.col;
                for j = 1:length(elvs)
                    [r, c] = dir.move1cell(r, c);
                    if ~isnan(elvs(j))
                        obj.elevation_map(r, c) = elvs(j);
                        obj.known_nodes(r, c) = obj.known_nodes(r, c) == 1;
                    end
                end
            end
        end

        function [] = senseObstacles(obj)
        % Calls the terrain methods to sense the obstacles around the current position
            for i = 1:4
                dir = DIRECTION(i);
                obss = obj.terrain.senseObstacles(obj.row, obj.col, dir, ...
                    obj.rig.obstacle_accuracy(i, :));
                r = obj.row;
                c = obj.col;
                for j = 1:length(obss)
                    [r, c] = dir.move1cell(r, c);
                    if ~isnan(obss(j))
                        obj.obstacle_map(r, c) = obss(j);
                        obj.known_nodes(r, c) = obj.known_nodes(r, c) == 1;
                    end
                end
            end
        end

        %% Display
        function [out] = toString(obj)
            out = append(sprintf('Robot Class'), newline);
            out = append(out, sprintf('\tEstimated Elevation:'), newline);

            elvs = string(obj.elevation_map);
            for i = 1:obj.terrain.n
                out = append(out, sprintf('\t'));
                for j = 1:obj.terrain.n
                    if ismissing(elvs(i, j)) 
                        out = append(out, sprintf('███ '));
                    else
                        out = append(out, sprintf('%.1f ', elvs(i, j)));
                    end
                end
                out = append(out, newline);
            end

            out = append(out, sprintf('\n\tEstimated Obstacles: \n'));
            obss = string(obj.obstacle_map);
            for i = 1:obj.terrain.n
                out = append(out, sprintf('\t'));
                for j = 1:obj.terrain.n
                    if ismissing(obss(i, j)) 
                        out = append(out, sprintf('█ '));
                    else
                        out = append(out, sprintf(obss(i, j) + sprintf(' ')));
                    end
                end
                out = append(out, newline);
            end
        end


    end
end