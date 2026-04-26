function [d, t, a] = computePointToPoints(obj, agentPos, targetPos)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        agentPos (1, 3) double;
        targetPos (:, 3) double;
    end
    arguments (Output)
        d (:, 1) double;
        t (:, 1) double;
        a (:, 1) double;
    end

    % distance from sensor to target
    d = vecnorm(agentPos - targetPos, 2, 2);

    % distance from sensor nadir to target nadir (i.e. distance ignoring altitude)
    x = vecnorm(agentPos(1:2) - targetPos(:, 1:2), 2, 2);
    
    % tilt angle (degrees) (-90, 0 (down), 90)
    t = (180 - atan2d(x, targetPos(:, 3) - agentPos(3)));

    % azimuth angle (degrees) (0 (+y) clockwise to 360)
    a = mod(atan2d(targetPos(:,1) - agentPos(1), targetPos(:,2) - agentPos(2)), 360);
end