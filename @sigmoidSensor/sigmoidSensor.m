classdef sigmoidSensor
    properties (SetAccess = private, GetAccess = public)
        % Sensor parameters
        alphaDist = NaN;
        betaDist = NaN;
        alphaTilt = NaN; % degrees
        betaTilt = NaN;
    end

    methods (Access = public)
        [obj]               = initialize(obj, alphaDist, betaDist, alphaTilt, betaTilt);
        [value]             = sensorPerformance(obj, agentPos, agentPan, agentTilt, targetPos);
        [f] = plotParameters(obj);
    end
    methods (Access = private)
        x = distanceMembership(obj, d);
        x = tiltMembership(obj, t);
    end
end