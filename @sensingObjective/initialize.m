function obj = initialize(obj, objectiveFunction, domain, discretizationStep, protectedRange, sensorPerformanceMinimum, objectiveMu, objectiveSigma)
    arguments (Input)
        obj (1,1) {mustBeA(obj, "sensingObjective")};
        objectiveFunction (1, 1) {mustBeA(objectiveFunction, "function_handle")};
        domain (1, 1) {mustBeGeometry};
        discretizationStep (1, 1) double = 1;
        protectedRange (1, 1) double = 1;
        sensorPerformanceMinimum (1, 1) double = 1e-6;
        objectiveMu (:, 2) double = NaN(1, 2);
        objectiveSigma (:, 2, 2) double = NaN(1, 2, 2);
    end
    arguments (Output)
        obj (1,1) {mustBeA(obj, "sensingObjective")};
    end

    obj.discretizationStep = discretizationStep;

    obj.sensorPerformanceMinimum = sensorPerformanceMinimum;
    
    obj.protectedRange = protectedRange;

    % Extract footprint limits
    xMin = min(domain.footprint(:, 1));
    xMax = max(domain.footprint(:, 1));
    yMin = min(domain.footprint(:, 2));
    yMax = max(domain.footprint(:, 2));

    xGrid = unique([xMin:obj.discretizationStep:xMax, xMax]);
    yGrid = unique([yMin:obj.discretizationStep:yMax, yMax]);
    
    % Store grid points for plotting later
    [obj.X, obj.Y] = meshgrid(xGrid, yGrid);

    % Evaluate function over grid points
    obj.values = reshape(objectiveFunction(obj.X, obj.Y), size(obj.X));
    
    % Normalize
    obj.values = obj.values ./ max(obj.values, [], "all");

    % store ground position
    idx = obj.values == 1;
    if any(isnan(objectiveMu))
        obj.groundPos = [obj.X(idx), obj.Y(idx)];
        obj.groundPos = obj.groundPos(1, 1:2); % for safety, in case 2 points are maximal (somehow)
    else
        obj.groundPos = objectiveMu;
    end
    obj.objectiveSigma = objectiveSigma;

    assert(domain.distance([obj.groundPos, ones(size(obj.groundPos, 1), 1) .* domain.center(3)]) > protectedRange, "Domain is crowding the sensing objective")
end