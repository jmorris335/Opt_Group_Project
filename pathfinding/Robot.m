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

        % manh_distance - nxn array of manhatten distances to the goal
        manh_distance {mustBeNonnegative}

        elevation_map double
        obstacle_map

        steps cell
        elevation_change double {mustBeNonnegative}
    end
    
    methods
        %% Construction
        function obj = Robot(rig, terrain)
        % Basic constructor
            obj.rig = rig;
            obj.position = [1, 1];
            obj.terrain = terrain;
            obj.elevation_map = NaN(terrain.n);
            obj.obstacle_map = NaN(terrain.n);
            obj.known_nodes = NaN(terrain.n);
            obj.steps = {[1, 1]};
            obj.elevation_change = 0;

            % Initialize starting values
            obj.elevation_map(1, 1) = terrain.getElevationAt(1, 1);
            obj.obstacle_map(1, 1) = terrain.getObstacleAt(1, 1);
            obj.known_nodes(1, 1) = 1;
            obj.calcManhDistance();
        end

        function [] = calcManhDistance(obj)
        % Calculates the Manhatten distance to the goal for every unvisited node
        %   Nodes that have obstacles are considered as Inf
            manh = @(r, c) abs(obj.terrain.n - r) + abs(obj.terrain.n - c);
            obj.manh_distance = ones(obj.terrain.n) * Inf;
            for i = 1:obj.terrain.n
                for j = 1:obj.terrain.n
                    if obj.known_nodes(i, j) == 0
                        obj.manh_distance(i, j) = manh(i, j);
                    end
                end
            end
            obj.manh_distance(obj.obstacle_map == 1) = Inf;
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
            out = length(obj.steps);
        end

        function [] = placeAt(obj, row, col)
        % Places the robot on an arbitrary node in the grid
            row = min(max(row, 0), obj.terrain.n);
            col = min(max(col, 0), obj.terrain.n);
            obj.position = [row, col];
        end

        function [out] = costOfPath(obj, beta)
        % Returns the cost of the path the robot has currently traveled
            out = length(obj.steps) + beta * obj.elevation_change;
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

        %% Pathfinding
        function [] = pathfind(obj)
            kill_at = 30;
            count = 0;
            while ~isequal([obj.row, obj.col], [obj.terrain.n, obj.terrain.n]) ...
                    && sum(isnan(obj.known_nodes), 'all') ~= 0 && count <= kill_at
                here = [obj.row, obj.col];
                there = obj.chooseNextStep();
                [there, closest_node] = obj.findOptimalPath(here, there);
                obj.travelPath(here, there, closest_node);
                count = count + 1;
            end
        end

        function [there] = chooseNextStep(obj)
        % Finds the closest, unvisited node to the end goal, as compared by
        % the manhatten distance
            obj.calcManhDistance();
            [there(1), there(2)] = find(obj.manh_distance == min(obj.manh_distance, [], 'all'), 1);
        end

        function [final, closest_node, costs] = findOptimalPath(obj, here, there)
        % Find the cheapest path from "from" to "there" using Djikstra's
        %   algorithm. Returns a nxn array of nodes detailing the cheapest
        %   neighbor, the costs to visit each node from "here", and the
        %   node that the path ends on ("current")
        %
        %   Note  that from and there are both 1x2 vectors (row, column)

            % Initialiation
            unchecked = ~isnan(obj.known_nodes);
            closest_node = cell(obj.terrain.n);
            costs = ones(obj.terrain.n) * Inf;
            costs(here(1), here(2)) = 0;
            current = here;
            final = here;

            % Run until goal node is fully checked
            while unchecked(there(1), there(2)) == 1
                unchecked(current(1), current(2)) = 0;

                % Explore edges (directional moves)
                for i = 1:4
                    [r, c] = DIRECTION(i).move1cell(current(1), current(2));
                    if (r > obj.terrain.n) || (c > obj.terrain.n) || (r <= 0) ...
                            || (c <= 0) || obj.obstacle_map(r, c) == 1 ...
                            || isnan(obj.known_nodes(r, c)) || unchecked(r, c) == 0
                        continue
                    else
                        this_cost = costs(current(1), current(2)) + 1 + ...
                            obj.elevation_map(r, c) - obj.elevation_map(current(1), current(2));
                        if this_cost < costs(r, c)
                            costs(r, c) = this_cost;
                            closest_node{r, c} = current;
                        end
                    end
                end

                % Break if all nodes are checked, otherwise set cheapest
                % node as current
                if sum(and(unchecked, ~isnan(obj.known_nodes)), 'all') == 0
                    break
                else
                    if costs(current(1), current(2)) ~= Inf
                        final = current;
                    end
                    [current(1), current(2)] = find(and(costs == min(costs(unchecked)), unchecked), 1);
                end
            end
        end
        
        function [] = travelPath(obj, here, there, closest_node)
        % Moves the robot along the path from "here" to "there" via the
        %   neighbor nodes defined in closest_node
        %
        %   Note that "here" and "there" are 1x2 arrays (row, column)
            if isequal(here, there) 
                return
            end
            path = {there};

            while ~isequal(closest_node{path{end}(1), path{end}(2)}, here)
                path{end+1} = closest_node{path{end}(1), path{end}(2)};
            end

            for i = 0:length(path) - 1
                if ~(obj.move(path{length(path) - i}))
                    return
                end
            end
        end

        function [succeeded] = move(obj, there)
        % move in a straight line from the robot's current position to "there"
        %   
        %   Note that there is a 1x2 array (row, column)
            while ~isequal([obj.row, obj.col], there)
                org_row = obj.row;
                org_col = obj.col;

                obj.position(1) = min(obj.terrain.n, obj.row + (there(1) > obj.row));
                obj.position(1) = max(0, obj.row - (there(1) < obj.row));
                obj.position(2) = min(obj.terrain.n, obj.col + (there(2) > obj.col));
                obj.position(2) = max(0, obj.col - (there(2) < obj.col));

                if obj.terrain.getObstacleAt(obj.row, obj.col)
                    obj.position(1) = org_row;
                    obj.position(2) = org_col;
                    succeeded = false;
                    return
                end

                obj.steps{end+1} = [obj.row, obj.col];
                obj.elevation_change = obj.elevation_change + ...
                    abs(obj.terrain.getElevationAt(obj.row, obj.col) ...
                    - obj.terrain.getElevationAt(org_row, org_col));

                obj.known_nodes(obj.row, obj.col) = 1;
                obj.exploreHere();
            end
            succeeded = true;
        end



        %% Display
        function [out] = plotPath(obj)
            xcoord = zeros(1, length(obj.steps));
            ycoord = zeros(1, length(obj.steps));
            for i = 1:length(obj.steps)
                xcoord(i) = obj.steps{i}(2);
                ycoord(i) = obj.steps{i}(1);
            end

            N = obj.terrain.n;

            hold on
            for i = 1:N
                for j = 1:N
                    if obj.terrain.getObstacleAt(j, i)
                        plot(i, N-j+1, 'xk', 'MarkerSize', 40);
                    else
                        plot(i, N-j+1, '.k', 'MarkerSize', 20);
                    end
                end
            end

            plot(xcoord, N-ycoord+1, 'r', 'LineWidth', 2);
            title('Path of Robot')
            subtitle(sprintf('Cost: %.2g', obj.costOfPath(1)));
            hold off
        end

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