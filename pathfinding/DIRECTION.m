classdef DIRECTION < uint8
    % DIRECTION - Defines the motion the robot can detect spaces in the
    % terrain, as well as move.

    enumeration
        UP (1)        % UP - current row - 1
        DWN (2)       % DOWN - current row + 1
        LFT (3)       % LEFT - current column - 1
        RHT (4)       % RIGHT - current column + 1
    end

    methods 
        function [new_row, new_col] = move1cell(obj, old_row, old_col)
            if obj == DIRECTION.UP
                new_row = max(0, old_row - 1);
                new_col = old_col;
            elseif obj == DIRECTION.DWN
                new_row = old_row + 1;
                new_col = old_col;
            elseif obj == DIRECTION.LFT
                new_row = old_row;
                new_col = max(0, old_col - 1);
            elseif obj == DIRECTION.RHT
                new_row = old_row;
                new_col = old_col + 1;
            end
        end
    end
end

