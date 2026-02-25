function obj = initializeWithValues(obj, values, domain, discretizationStep, protectedRange, sensorPerformanceMinimum)
    arguments (Input)
        obj (1,1) {mustBeA(obj, "sensingObjective")};
        values (:,:) double;
        domain (1, 1) {mustBeGeometry};
        discretizationStep (1, 1) double = 1;
        protectedRange (1, 1) double = 1;
        sensorPerformanceMinimum (1, 1) double = 1e-6;
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

    % Store grid points
    [obj.X, obj.Y] = meshgrid(xGrid, yGrid);

    % Use pre-computed values (caller must evaluate on same grid)
    obj.values = reshape(values, size(obj.X));

    % Normalize
    obj.values = obj.values ./ max(obj.values, [], "all");

    % Store ground position (peak of objective)
    idx = obj.values == 1;
    obj.groundPos = [obj.X(idx), obj.Y(idx)];
    obj.groundPos = obj.groundPos(1, 1:2);

    assert(domain.distance([obj.groundPos, domain.center(3)]) > protectedRange, "Domain is crowding the sensing objective")
end
