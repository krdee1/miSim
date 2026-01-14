classdef sensingObjective
    % Sensing objective definition parent class
    properties (SetAccess = private, GetAccess = public)
        label = "";
        groundAlt = NaN;
        groundPos = [NaN, NaN];
        discretizationStep = NaN;
        objectiveFunction = @(x, y) NaN; % define objective functions over a grid in this manner
        X = [];
        Y = [];
        values = [];
        protectedRange = NaN; % keep obstacles from crowding objective
        sensorPerformanceMinimum = NaN; % minimum sensor performance to allow assignment of a point in the domain to a partition
    end

    methods (Access = public)
        [obj] = initialize(obj, objectiveFunction, domain, discretizationStep, protectedRange, sensorPerformanceMinimum);
        [obj] = initializeRandomMvnpdf(obj, domain, protectedRange, discretizationStep, protectedRange);
        [f  ] = plot(obj, ind, f);
    end
end