classdef sensingObjective
    % Sensing objective definition parent class
    properties (SetAccess = private, GetAccess = public)
        label = "";
        groundAlt = 0;
        groundPos = [0, 0];
        discretizationStep = 1;
        objectiveFunction = @(x, y) 0; % define objective functions over a grid in this manner
        X = [];
        Y = [];
        values = [];
        protectedRange = 1; % keep obstacles from crowding objective
        sensorPerformanceMinimum = 1e-6; % minimum sensor performance to allow assignment of a point in the domain to a partition
    end

    methods (Access = public)
        [obj] = initialize(obj, objectiveFunction, domain, discretizationStep, protectedRange, sensorPerformanceMinimum);
        [obj] = initializeRandomMvnpdf(obj, domain, protectedRange, discretizationStep, protectedRange);
        [f  ] = plot(obj, ind, f);
    end
end