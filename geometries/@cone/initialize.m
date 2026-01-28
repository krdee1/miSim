function obj = initialize(obj, center, radius, height, tag, label)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "cone")};
        center (1, 3) double;
        radius (1, 1) double;
        height (1, 1) double;
        tag (1, 1) REGION_TYPE = REGION_TYPE.INVALID;
        label (1, 1) string = "";
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "cone")};
    end

    obj.center = center;
    obj.radius = radius;
    obj.height = height;
    obj.tag = tag;
    obj.label = label;
end