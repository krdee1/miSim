function [obj] = initializeRandom(obj, minDimension, tag, label)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
        minDimension (1, 1) double = 10;
        tag (1, 1) REGION_TYPE = REGION_TYPE.INVALID;
        label (1, 1) string = "";
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
    end
    
    % Produce random bounds
    L = ceil(minDimension + rand * minDimension);
    bounds = [zeros(1, 3); L * ones(1, 3)];

    % Regular initialization
    obj = obj.initialize(bounds, tag, label);
end