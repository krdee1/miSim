classdef spherical
    % Rectangular prism geometry
    properties (SetAccess = private, GetAccess = public)
        % Meta
        tag = REGION_TYPE.INVALID;
        label = "";

        % Spatial
        center = NaN;
        radius = NaN;
        diameter = NaN;

        vertices; % fake vertices
        edges; % fake edges

        % Plotting
        lines;

        % collision
        barrierFunction;
        dBarrierFunction;
    end
    properties (SetAccess = public, GetAccess = public)
        % Sensing objective (for DOMAIN region type only)
        objective;
    end

    methods (Access = public)
        [obj   ] = initialize(obj, center, radius, tag, label);
        [r     ] = random(obj);
        [c     ] = contains(obj, pos);
        [d     ] = distance(obj, pos);
        [g     ] = distanceGradient(obj, pos);
        [c     ] = containsLine(obj, pos1, pos2);
        [obj, f] = plotWireframe(obj, ind, f);
    end
end