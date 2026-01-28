function cPos = closestToPoint(obj, pos)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rectangularPrism")};
        pos (:, 3) double;
    end
    arguments (Output)
        cPos (:, 3) double;
    end
    cPos = NaN(1, 3);
    for ii = 1:3
        if pos(ii) < obj.minCorner(ii)
            cPos(ii) = obj.minCorner(ii);
        elseif pos(ii) > obj.maxCorner(ii)
            cPos(ii) = obj.maxCorner(ii);
        else
            cPos(ii) = pos(ii);
        end
    end
end