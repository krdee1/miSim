function g = distanceGradient(obj, pos)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
        pos (:, 3) double;
    end
    arguments (Output)
        g (:, 3) double
    end
    
    % find nearest point on surface to query position
    q = min(max(pos, obj.minCorner), obj.maxCorner);

    % Find distance and direction between pos and q
    v = pos - q;
    vNorm = norm(v);

    % position is outside geometry
    if vNorm > 0
        % gradient is normalized vector from q to p
        g = v / vNorm;
        return;
    end

    % position is on or in geometry
    % find distances to each face in each dimension
    distances = [pos(1) - obj.minCorner(1), obj.maxCorner(1) - pos(1), pos(2) - obj.minCorner(2), obj.maxCorner(2) - pos(2), pos(3) - obj.minCorner(3), obj.maxCorner(3) - pos(3)];
    [~, idx] = min(distances);

    % I think there needs to be additional handling here for the
    % edge/corner cases, where there are ways to balance or resolve ties
    % when two faces are equidistant to the query position
    assert(sum(idx) == idx, "Implement edge case handling");

    % select gradient that brings us quickest to the nearest face
    g = [ 1,  0,  0; ...
         -1,  0,  0; ...
          0,  1,  0; ...
          0, -1,  0; ...
          0,  0,  1; ...
          0,  0, -1;];
    g = g(idx, :);
end