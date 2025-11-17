classdef sigmoidSensor
    properties (SetAccess = private, GetAccess = public)
        % Sensor parameters
        alphaDist = NaN;
        betaDist = NaN;
        alphaPan = NaN;
        betaPan = NaN;
        alphaTilt = NaN;
        betaTilt = NaN;
    end

    methods (Access = public)
        [obj]               = initialize(obj, alphaDist, betaDist, alphaPan, betaPan, alphaTilt, betaTilt);
        [values, positions] = sense(obj, agent, sensingObjective, domain, partitioning);
        [value]             = sensorPerformance(obj, agentPos, agentPan, agentTilt, targetPos);
        plotParameters(obj);
    end
    methods (Access = private)
        x = distanceMembership(obj, d);
        x = tiltMembership(obj, t);
    end
end