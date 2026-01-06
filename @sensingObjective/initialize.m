function obj = initialize(obj, objectiveFunction, domain, discretizationStep, protectedRange)
    arguments (Input)
        obj (1,1) {mustBeA(obj, 'sensingObjective')};
        objectiveFunction (1, 1) {mustBeA(objectiveFunction, 'function_handle')};
        domain (1, 1) {mustBeGeometry};
        discretizationStep (1, 1) double = 1;
        protectedRange (1, 1) double = 1;
    end
    arguments (Output)
        obj (1,1) {mustBeA(obj, 'sensingObjective')};
    end

    obj.discretizationStep = discretizationStep;

    obj.groundAlt = domain.minCorner(3);
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
    obj.objectiveFunction = objectiveFunction;
    obj.values = reshape(obj.objectiveFunction(obj.X, obj.Y), size(obj.X));
    
    % Normalize
    obj.values = obj.values ./ max(obj.values, [], "all");

    % store ground position
    idx = obj.values == 1;
    obj.groundPos = [obj.X(idx), obj.Y(idx)];
    obj.groundPos = obj.groundPos(1, 1:2); % for safety, in case 2 points are maximal (somehow)

    assert(domain.distance([obj.groundPos, domain.center(3)]) > protectedRange, "Domain is crowding the sensing objective")
end