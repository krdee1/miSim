function c = containsLine(obj, pos1, pos2)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'spherical')};
        pos1 (1, 3) double;
        pos2 (1, 3) double;
    end
    arguments (Output)
        c (1, 1) logical
    end
    
    d = pos2 - pos1;
    f = pos1 - obj.center;

    a = dot(d, d);
    b = 2 * dot(f, d);
    c = dot(f, f) - obj.radius^2;

    disc = b^2 - 4*a*c;

    if disc < 0
        c = false;
        return;
    end

    t = [(-b - sqrt(disc)) / (2 * a), (-b + sqrt(disc)) / (2 * a)];

    c = (t(1) >= 0 && t(1) <= 1) || (t(2) >= 0 && t(2) <= 1);
end