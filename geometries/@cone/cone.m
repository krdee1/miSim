classdef cone
    % Conical geometry
    properties (SetAccess = private, GetAccess = public)
        % Meta
        tag = REGION_TYPE.INVALID;
        label = "";

        % Spatial
        center = NaN;
        radius = NaN;
        height = NaN;

        % Plotting
        surface;
        n = 32;
    end

    methods (Access = public)
        [obj   ] = initialize(obj, center, radius, height, tag, label);
        [obj, f] = plot(obj, ind, f);
    end
end