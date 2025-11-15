function d = distance(obj, pos)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
        pos (:, 3) double;
    end
    arguments (Output)
        d (:, 1) double
    end
    if obj.contains(pos)
        % Queried point is inside geometry
        % find minimum distance to any face
        d = min([pos(1) - obj.minCorner(1), ...
                 pos(2) - obj.minCorner(2), ...
                 pos(3) - obj.minCorner(3), ...
                 obj.maxCorner(1) - pos(1), ...
                 obj.maxCorner(2) - pos(2), ...
                 obj.maxCorner(3) - pos(3)]);
    else
        % Queried point is outside geometry
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
        d = norm(cPos - pos);
    end
end