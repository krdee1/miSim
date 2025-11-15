function c = contains(obj, pos)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
        pos (:, 3) double;
    end
    arguments (Output)
        c (:, 1) logical
    end
    c = all(pos >= repmat(obj.minCorner, size(pos, 1), 1), 2) & all(pos <= repmat(obj.maxCorner, size(pos, 1), 1), 2);
end