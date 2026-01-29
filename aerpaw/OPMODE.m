classdef OPMODE
    properties (Constant)
        INVALID      = uint8(0);
        INITIALIZED  = uint8(1);
        SET          = uint8(2);
        RUNNING      = uint8(3);
        CONCLUDING   = uint8(4);
        FINISHED     = uint8(5);
    end
end
