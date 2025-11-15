function r = random(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
    end
    arguments (Output)
        r (1, 3) double
    end
    r = (obj.vertices(1, 1:3) + rand(1, 3) .* obj.vertices(8, 1:3) - obj.vertices(1, 1:3))';
end