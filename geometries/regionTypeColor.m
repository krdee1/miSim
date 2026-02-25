function color = regionTypeColor(tag)
% regionTypeColor  Return the RGB color (0-1 range) for a REGION_TYPE value.
arguments (Input)
    tag (1, 1) REGION_TYPE
end
arguments (Output)
    color (1, 3) double
end

switch tag
    case REGION_TYPE.DOMAIN
        color = [0, 0, 0] / 255;
    case REGION_TYPE.OBSTACLE
        color = [255, 127, 127] / 255;
    case REGION_TYPE.COLLISION
        color = [255, 255, 128] / 255;
    case REGION_TYPE.FOV
        color = [255, 165, 0] / 255;
    case REGION_TYPE.COMMS
        color = [0, 255, 0] / 255;
    otherwise % INVALID
        color = [255, 127, 255] / 255;
end
end