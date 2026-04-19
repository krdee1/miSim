function SINR = sensorPerformance(obj, agentPos, targetPos, otherSensors, otherSensorsPos)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        agentPos (1, 3) double;
        targetPos (1, 3) double;
        otherSensors (:, 1) cell = {};
        otherSensorsPos (:, 3) double = NaN(0, 3);
    end
    arguments (Output)
        SINR (:, 1) double;
    end
    assert(size(otherSensors, 1) == size(otherSensorsPos, 1), "Mismatch in length of other sensors (%d) and their positions (%d)", size(otherSensors, 1), size(otherSensorsPos, 1));

    d = vecnorm(agentPos - targetPos, 2, 2); % distance from sensor to target
    d_other = NaN(size(otherSensors));

    x = vecnorm(agentPos(1:2) - targetPos(1, 1:2), 2, 2); % distance from sensor nadir to target nadir (i.e. distance ignoring height difference)
    x_other = NaN(size(otherSensors));

    t = (180 - atan2d(x, targetPos(1, 3) - agentPos(3))); % degrees
    t_other = NaN(size(otherSensors));

    % Performance is measured as SINR for this sensor
    S = 10^(0.1 * obj.RSS(d, t)); % Signal
    N = obj.k_B * obj.T_0 * obj.BW; % Thermal noise
    I = 0; % Interference from other agents
    for ii = 1:size(otherSensors, 1)
        d_other(ii, 1) = vecnorm(otherSensorsPos(ii, 1:3) - targetPos, 2, 2);

        x_other(ii, 1) = vecnorm(otherSensorsPos(ii, 3) - targetPos(1, 1:2), 2, 2);
        t_other(ii, 1) = (180 - atan2d(x_other(ii, 1), targetPos(1, 3) - otherSensorsPos(ii, 3)));
        
        I = I + 10 ^ (0.1 * otherSensors.RSS(otherSensors{ii}, d_other(ii), t_other(ii)));
    end
    SINR = 10*log10(S/(I + N));
end