classdef PsuedoRandom < handle
    % PsuedoRandom generates a list of psuedorandom numbers that can be
    % called to guarantee a deterministic random number sequence

    properties
        at {mustBePositive, mustBeInteger} % int, current index for random number
        seed {mustBePositive, mustBeInteger} % seed for random number generator
        length {mustBePositive} % length of random number array
        pr_floats double {mustBeNonnegative} % lengthx1 array of random floats (0 to 1)
    end

    methods
        function obj = PsuedoRandom(varargin)
        % Constructor. Seed can be specified if desired.
            if nargin > 0
                obj.seed = varargin(1);
            else
                obj.seed = 1;
            end

            obj.at = 1;
            obj.length = 1000;
            rng(obj.seed);
            obj.pr_floats = rand(obj.length, 1);
        end

        function [] = increment(obj)
        % Advances "at" index by one or wraps
            if obj.at >= obj.length
                obj.at = 1;
            else
                obj.at = obj.at + 1;
            end
        end

        function [] = set(obj, new_at)
        % set the "at" index to a new position
            if new_at > obj.length || new_at < 1
                obj.at = 1;
            else
                obj.at = new_at;
            end
        end

        function [pr_float] = getFloat(obj)
        % Get a psuedo-random float number
            pr_float = obj.pr_floats(obj.at);
            obj.increment();
        end

        function [pr_int] = getInt(obj, upper_bound)
        % get a pseudo-random int number between 0 and upper_bound
            pr_int = floor(obj.pr_ints(obj.at) * upper_bound(1));
            obj.increment();
        end
    end
end