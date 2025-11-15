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
    tiltAngle = atan2(targetPos(:, 3) - agentPos(3), x) - agentTilt;

    % Membership functions
    mu_d = 1 - (1 ./ (1 + exp(-obj.betaDist .* (d - obj.alphaDist)))); % distance
    mu_p = 1; % pan
    mu_t = (1 ./ (1 + exp(-obj.betaPan .* (tiltAngle + obj.alphaPan)))) - (1 ./ (1 + exp(-obj.betaPan .* (tiltAngle - obj.alphaPan)))); % tilt

    value = mu_d .* mu_p .* mu_t * 1e12;
end