function r = random(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "spherical")};
    end
    arguments (Output)
        r (1, 3) double
    end
    y = (rand - 0.5) * 2; % uniform draw on [-1, 1]
    R = sqrt(1 - y^2);
    lon = (rand - 0.5) * 2 * pi; % uniform draw on [-pi, pi]
    s = [R * sin(lon), y, R * cos(lon)]; % random point on surface
    r = s * rand^(1/3); % scaled to random normalized radius [0, 1]

    r = obj.center + obj.radius * r;
end