classdef rectangularPrism
    % Rectangular prism geometry
    properties (SetAccess = private, GetAccess = public)
        % Meta
        tag = REGION_TYPE.INVALID;
        label = "";

        % Spatial
        minCorner = NaN(1, 3);
        maxCorner = NaN(1, 3);
        dimensions = NaN(1, 3);
        center = NaN;
        footprint = NaN(4, 2);

        % Graph
        vertices = NaN(8, 3);
        edges = [1 2; 2 3; 3 4; 4 1;  % bottom square
            5 6; 6 8; 8 7; 7 5;  % top square
            1 5; 2 6; 3 8; 4 7]; % vertical edges

        % Plotting
        lines;
    end
    properties (SetAccess = public, GetAccess = public)
        % Sensing objective (for DOMAIN region type only)
        objective;
    end

    methods (Access = public)
        [obj   ] = initialize(obj, bounds, tag, label, objectiveFunction, discretizationStep);
        [obj   ] = initializeRandom(obj, tag, label, minDimension, maxDimension, domain);
        [r     ] = random(obj);
        [c     ] = contains(obj, pos);
        [d     ] = distance(obj, pos);
        [c     ] = containsLine(obj, pos1, pos2);
        [obj, f] = plotWireframe(obj, ind, f);
    end
end