classdef fixedCardinalSensor
    % Senses in the +/-x, +/- y directions at some specified fixed length
    properties
        alphaTilt = NaN;
        r = 0.1; % fixed sensing length
    end

    methods (Access = public)
        [obj] = initialize(obj, r);
        [neighborValues, neighborPos] = sense(obj, agent, sensingObjective, domain, partitioning);
        [value] = sensorPerformance(obj, agentPos, agentPan, agentTilt, targetPos);
    end
end