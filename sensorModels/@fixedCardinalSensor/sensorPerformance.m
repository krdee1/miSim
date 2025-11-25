function value = sensorPerformance(obj, agentPos, agentPan, agentTilt, targetPos)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'fixedCardinalSensor')};
        agentPos (1, 3) double;
        agentPan (1, 1) double;
        agentTilt (1, 1) double;
        targetPos (:, 3) double;
    end
    arguments (Output)
        value (:, 1) double;
    end

    value = 0.5 * ones(size(targetPos, 1), 1);
end