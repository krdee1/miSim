classdef cone
    % Conical geometry
    properties (SetAccess = private, GetAccess = public)
        % Meta
        tag = REGION_TYPE.INVALID;
        label = "";

        % Spatial
        center  = NaN;
        radius  = NaN;
        height  = NaN;
        tilt    = 0;   % degrees, 0=nadir 90=horizon
        azimuth = 0;   % degrees, 0=+Y 90=+X clockwise

        % Plotting
        surface;
        n = 32;
    end

    methods (Access = public)
        [obj   ] = initialize(obj, center, radius, height, tag, label);
        [obj, f] = plot(obj, ind, f, maxAlt);
    end
end