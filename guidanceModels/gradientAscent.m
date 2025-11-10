function nextPos = gradientAscent(sensedValues, sensedPositions, pos)
    arguments (Input)
        sensedValues (:, 1) double;
        sensedPositions (:, 3) double;
        pos (1, 3) double;
    end
    arguments (Output)
        nextPos(1, 3) double;
    end
    
    % Select next position by maximum sensed value
    nextPos = sensedPositions(sensedValues == max(sensedValues), :);
    nextPos = [nextPos(1, 1:2), pos(3)]; % just in case two get selected, simply pick one
end