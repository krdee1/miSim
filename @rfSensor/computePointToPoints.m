function [d, t] = computePointToPoints(obj, agentPos, targetPos)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        agentPos (1, 3) double;
        targetPos (:, 3) double;
    end
    arguments (Output)
        d (:, 1) double;
        t (:, 1) double;
    end

    % distance from sensor to target
    d = vecnorm(agentPos - targetPos, 2, 2);

    % distance from sensor nadir to target nadir (i.e. distance ignoring altitude)
    x = vecnorm(agentPos(1:2) - targetPos(:, 1:2), 2, 2);
    
    % tilt angle (degrees)
    t = (180 - atan2d(x, targetPos(:, 3) - agentPos(3)));

end