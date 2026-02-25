classdef spherical
    % Rectangular prism geometry
    properties (SetAccess = private, GetAccess = public)
        % Spatial
        center = NaN(1, 3);
        radius = NaN;
        diameter = NaN;

        vertices = NaN(6, 3); % fake vertices
        edges = NaN(8, 2); % fake edges

        % Plotting
        lines;
    end
    properties (SetAccess = public, GetAccess = public)
        % Meta
        tag = REGION_TYPE.INVALID;
        label = "";
        
        % Sensing objective (for DOMAIN region type only)
        objective;
    end

    methods (Access = public)
        function obj = spherical()
            arguments (Output)
                obj (1, 1) spherical
            end
            obj.objective = sensingObjective;
        end
        [obj   ] = initialize(obj, center, radius, tag, label);
        [r     ] = random(obj);
        [c     ] = contains(obj, pos);
        [d     ] = distance(obj, pos);
        [g     ] = distanceGradient(obj, pos);
        [c     ] = containsLine(obj, pos1, pos2);
        [obj, f] = plotWireframe(obj, ind, f);
    end
end