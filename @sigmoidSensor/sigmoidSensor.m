classdef sigmoidSensor
    properties (SetAccess = private, GetAccess = public)
        % Sensor parameters
        alphaDist = NaN;
        betaDist = NaN;
        alphaTilt = NaN; % degrees
        betaTilt = NaN;
    end

    methods (Access = public)
        [obj]   = initialize(obj, alphaDist, betaDist, alphaTilt, betaTilt); % initialize sensor, define parameters
        [value] = sensorPerformance(obj, agentPos, targetPos); % determine sensor performance for a given single sensor and target geometry
        [f]     = plotParameters(obj); % debug, plot sensor response as a function of distance and tilt angle
    end
    methods (Access = private)
        x = distanceMembership(obj, d); % used in computing distance factor of sensor performance
        x = tiltMembership(obj, t); % used in computing tilt factor of sensor performance
    end
end