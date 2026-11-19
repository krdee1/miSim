function d = distance(obj, pos)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rectangularPrism")};
        pos (:, 3) double;
    end
    arguments (Output)
        d (:, 1) double;
    end
    % Evaluate each queried point separately. Edited by Reza Barzegaran because linear indexing of a multi-row pos mixed coordinates of different points.
    d = NaN(size(pos, 1), 1);
    for kk = 1:size(pos, 1)
        p = pos(kk, :);
        if obj.contains(p)
            % Queried point is inside geometry
            % find minimum distance to any face
            d(kk) = min([p(1) - obj.minCorner(1), ...
                     p(2) - obj.minCorner(2), ...
                     p(3) - obj.minCorner(3), ...
                     obj.maxCorner(1) - p(1), ...
                     obj.maxCorner(2) - p(2), ...
                     obj.maxCorner(3) - p(3)]);
        else
            % Queried point is outside geometry
            cPos = NaN(1, 3);
            for ii = 1:3
                if p(ii) < obj.minCorner(ii)
                    cPos(ii) = obj.minCorner(ii);
                elseif p(ii) > obj.maxCorner(ii)
                    cPos(ii) = obj.maxCorner(ii);
                else
                    cPos(ii) = p(ii);
                end
            end
            d(kk) = norm(cPos - p);
        end
    end
end
