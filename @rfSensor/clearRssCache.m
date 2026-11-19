function obj = clearRssCache(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
    end

    cache = zeros(0, 1);
    coder.varsize('cache', [Inf, 1], [1, 0]);
    obj.rssCache = cache;

end