classdef REGION_TYPE
    properties
        id
        color
    end
    enumeration
        INVALID (0, [255, 127, 255]); % default value
        DOMAIN (1, [0, 0, 0]); % domain region
        OBSTACLE (2, [255, 127, 127]); % obstacle region
        COLLISION (3, [255, 255, 128]); % collision avoidance region
    end
    methods
        function obj = REGION_TYPE(id, color)
            obj.id = id;
            obj.color = color./ 255;
        end
    end
end