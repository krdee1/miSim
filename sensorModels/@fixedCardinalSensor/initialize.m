function obj = initialize(obj, r)
    arguments(Input)
        obj (1, 1) {mustBeA(obj, 'fixedCardinalSensor')};
        r (1, 1) double;
    end
    arguments(Output)
        obj (1, 1) {mustBeA(obj, 'fixedCardinalSensor')};
    end
    obj.r = r;
end