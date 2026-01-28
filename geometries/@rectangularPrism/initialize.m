function obj = initialize(obj, bounds, tag, label, objectiveFunction, discretizationStep)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rectangularPrism")};
        bounds (2, 3) double;
        tag (1, 1) REGION_TYPE = REGION_TYPE.INVALID;
        label (1, 1) string = "";
        objectiveFunction (1, 1) function_handle = @(x, y) 1;
        discretizationStep (1, 1) double = 1;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "rectangularPrism")};
    end

    obj.tag = tag;
    obj.label = label;

    % Define geometry bounds by LL corner and UR corner
    obj.minCorner = bounds(1, 1:3);
    obj.maxCorner = bounds(2, 1:3);

    % Compute L, W, H
    obj.dimensions = [obj.maxCorner(1) - obj.minCorner(1), obj.maxCorner(2) - obj.minCorner(2), obj.maxCorner(3) - obj.minCorner(3)];

    % Compute center
    obj.center = obj.minCorner + obj.dimensions ./ 2;

    % Compute a (fake) radius
    % fully contains the rectangular prism from the center
    obj.radius = (1/2) * sqrt(sum(obj.dimensions.^2));

    % Compute vertices
    obj.vertices = [obj.minCorner;
                    obj.maxCorner(1), obj.minCorner(2:3);
                    obj.maxCorner(1:2), obj.minCorner(3);
                    obj.minCorner(1), obj.maxCorner(2), obj.minCorner(3);
                    obj.minCorner(1:2), obj.maxCorner(3);
                    obj.maxCorner(1), obj.minCorner(2), obj.maxCorner(3);
                    obj.minCorner(1), obj.maxCorner(2:3)
                    obj.maxCorner;];

    % Compute footprint
    obj.footprint = [obj.minCorner(1:2); ...
                     [obj.minCorner(1), obj.maxCorner(2)]; ...
                     [obj.maxCorner(1), obj.minCorner(2)]; ...
                     obj.maxCorner(1:2)];

    % Instantiate sensingObjective only for DOMAIN-type regions
    if tag == REGION_TYPE.DOMAIN
        obj.objective = sensingObjective;
    end
end