classdef REGION_TYPE < uint8
    enumeration
        INVALID   (0) % default value
        DOMAIN    (1) % domain region
        OBSTACLE  (2) % obstacle region
        COLLISION (3) % collision avoidance region
        FOV       (4) % field of view region
        COMMS     (5) % comunications region
    end
end
