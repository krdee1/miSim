classdef sensingObjective
    % Sensing objective definition parent class
    properties (SetAccess = private, GetAccess = public)
        label = "";
        groundPos = NaN(1, 2);
        objectiveSigma = NaN(1, 2, 2);
        discretizationStep = NaN;
        X = [];
        Y = [];
        values = [];
        protectedRange = NaN; % keep obstacles from crowding objective
        sensorPerformanceMinimum = NaN; % minimum sensor performance to allow assignment of a point in the domain to a partition
    end

    methods (Access = public)
        [obj] = initialize(obj, objectiveFunction, domain, discretizationStep, protectedRange, sensorPerformanceMinimum);
        [obj] = initializeWithValues(obj, values, domain, discretizationStep, protectedRange, sensorPerformanceMinimum);
        [obj] = initializeRandomMvnpdf(obj, domain, discretizationStep, protectedRange);
        [f  ] = plot(obj, ind, f);
    end
end