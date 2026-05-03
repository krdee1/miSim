function value = halfAngle(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "sigmoidSensor")};
    end
    arguments (Output)
        value (1, 1) double;
    end
    value = obj.alphaTilt;
end
