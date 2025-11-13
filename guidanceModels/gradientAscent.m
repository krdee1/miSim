function nextPos = gradientAscent(sensedValues, sensedPositions, pos, rate)
    arguments (Input)
        sensedValues (:, 1) double;
        sensedPositions (:, 3) double;
        pos (1, 3) double;
        rate (1, 1) double = 0.1;
    end
    arguments (Output)
        nextPos(1, 3) double;
    end

    % As a default, maintain current position
    if size(sensedValues, 1) == 0 && size(sensedPositions, 1) == 0
        nextPos = pos;
        return;
    end
    
    % Select next position by maximum sensed value
    nextPos = sensedPositions(sensedValues == max(sensedValues), :);
    nextPos = [nextPos(1, 1:2), pos(3)]; % just in case two get selected, simply pick one

    % rate-limit motion
    v = nextPos - pos;
    nextPos = pos + (v / norm(v, 2)) * rate;

end