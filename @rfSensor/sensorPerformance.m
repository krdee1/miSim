function SINR = sensorPerformance(obj, agentPos, targetPos, otherSensorsPos, otherSensors)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        agentPos (1, 3) double;
        targetPos (:, 3) double;
        otherSensorsPos (:, 3) double = [];
        otherSensors (:, 1) cell = {};
    end
    arguments (Output)
        SINR (:, 1) double;
    end
    assert(size(otherSensorsPos, 1) == size(otherSensors, 1), "Mismatch in number of other sensor positions (%d) and number of other sensors (%d) provided", size(otherSensorsPos, 1), size(otherSensors, 1));

    [d, t, a] = obj.computePointToPoints(agentPos, targetPos);

    % Performance is measured as SINR for this sensor
    S = 10 .^ (0.1 .* obj.RSS(d, t, a)); % Signal
    I = zeros(size(d)); % Interference from other agents
    for ii = 1:size(otherSensors, 2)
        [d_other, t_other, a_other] = otherSensors{ii}.computePointToPoints(otherSensorsPos(ii, 1:3), targetPos);
        I = I + 10 .^ (0.1 .* otherSensors{ii}.RSS(d_other, t_other, a_other));
    end

    SINR = 10*log10(S ./ (I + obj.N));
end