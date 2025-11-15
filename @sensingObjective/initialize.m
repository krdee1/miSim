function obj = initialize(obj, objectiveFunction, footprint, groundAlt, discretizationStep)
    arguments (Input)
        obj (1,1) {mustBeA(obj, 'sensingObjective')};
        objectiveFunction (1, 1) {mustBeA(objectiveFunction, 'function_handle')};
        footprint (:, 2) double;
        groundAlt (1, 1) double = 0;
        discretizationStep (1, 1) double = 1;
    end
    arguments (Output)
        obj (1,1) {mustBeA(obj, 'sensingObjective')};
    end

    obj.groundAlt = groundAlt;

    % Extract footprint limits
    xMin = min(footprint(:, 1));
    xMax = max(footprint(:, 1));
    yMin = min(footprint(:, 2));
    yMax = max(footprint(:, 2));

    xGrid = unique([xMin:discretizationStep:xMax, xMax]);
    yGrid = unique([yMin:discretizationStep:yMax, yMax]);
    
    % Store grid points for plotting later
    [obj.X, obj.Y] = meshgrid(xGrid, yGrid);

    % Evaluate function over grid points
    obj.objectiveFunction = objectiveFunction;
    obj.values = reshape(obj.objectiveFunction(obj.X, obj.Y), size(obj.X));

    % store ground position
    idx = obj.values == max(obj.values, [], "all");
    obj.groundPos = [obj.X(idx), obj.Y(idx)];
end