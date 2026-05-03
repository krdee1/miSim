function obj = clearRssCache(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
    end

    obj.rssCache = double.empty(0, 1);

end