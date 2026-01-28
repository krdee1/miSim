function c = contains(obj, pos)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "spherical")};
        pos (:, 3) double;
    end
    arguments (Output)
        c (:, 1) logical
    end
    c = norm(obj.center - pos) <= obj.radius;
end