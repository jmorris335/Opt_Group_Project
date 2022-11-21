classdef Terrain
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
        
        topography_map  % nxn array of the elevation of the terrain
        obstacle_map    % nxn array of booleans, 0 is pass, 1 is no pass

        % Environment variables
        
        visibility      % normalized value, 1 is full vision, 0 is no vision
        
    end
    
    properties (Access=private)
        EDGE_LENGTH     % Length of one side of the map
        n               % Amount of pixels per side (precision)
    end
    
    methods
        %% Construction
        function obj = Terrain(edge_length, precision)
        % Basic constructor for class
            obj.EDGE_LENGTH = edge_length;
            obj.n = precision;
            obj.topography_map = createTopography();
            obj.obstacle_map = createObstacles();
        end
        
        function [topography_out] = createTopography(obj)
        % Returns a randomized map of the topography
            topography_out = zeros(obj.n, obj.n);
        end
        
        function [obstacles_out] = createObstacles(obj)
        % Returns a randomized map of the obstacles in the terrain
            obstacles_out = zeros(obj.n, obj.n);
        end 
    end
end