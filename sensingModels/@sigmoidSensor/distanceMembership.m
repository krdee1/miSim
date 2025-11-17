function x = distanceMembership(obj, d)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'sigmoidSensor')};
        d (:, 1) double;
    end
    arguments (Output)
        x (:, 1) double;
    end
    x = 1 - (1 ./ (1 + exp(-obj.betaDist .* (abs(d) - obj.alphaDist))));
end