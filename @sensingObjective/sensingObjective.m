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
    end

    methods (Access = public)
        [obj] = initialize(obj, objectiveFunction, domain, discretizationStep);
        [obj] = initializeRandomMvnpdf(obj, domain, protectedRange, discretizationStep);
        [f  ] = plot(obj, ind, f);
    end
end