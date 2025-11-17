function x = tiltMembership(obj, t)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'sigmoidSensor')};
        t (:, 1) double;
    end
    arguments (Output)
        x (:, 1) double;
    end
    x = (1 ./ (1 + exp(-obj.betaTilt .* (t + obj.alphaTilt)))) - (1 ./ (1 + exp(-obj.betaTilt .* (t - obj.alphaTilt))));
end