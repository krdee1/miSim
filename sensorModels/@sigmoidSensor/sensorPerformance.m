function value = sensorPerformance(obj, agentPos, agentPan, agentTilt, targetPos)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'sigmoidSensor')};
        agentPos (1, 3) double;
        agentPan (1, 1) double;
        agentTilt (1, 1) double;
        targetPos (:, 3) double;
    end
    arguments (Output)
        value (:, 1) double;
    end

    d = vecnorm(agentPos - targetPos, 2, 2); % distance from sensor to target
    x = vecnorm(agentPos(1:2) - targetPos(:, 1:2), 2, 2); % distance from sensor nadir to target nadir (i.e. distance ignoring height difference)
    tiltAngle = (180 - atan2d(x, targetPos(:, 3) - agentPos(3))) - agentTilt; % degrees

    % Membership functions
    mu_d = obj.distanceMembership(d);
    mu_t = obj.tiltMembership(tiltAngle);

    value = mu_d .* mu_t; % assume pan membership is always 1
end