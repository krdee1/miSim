function c = containsLine(obj, pos1, pos2)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
        pos1 (1, 3) double;
        pos2 (1, 3) double;
    end
    arguments (Output)
        c (1, 1) logical
    end
    
    d = pos2 - pos1;
    
    % edge case where the line is parallel to the geometry
    if abs(d) < 1e-12
        % check if it happens to start or end inside or outside of
        % the geometry
        if obj.contains(pos1) || obj.contains(pos2)
            c = true;
        else
            c = false;
        end
        return;
    end
    
    tmin = -inf;        
    tmax = inf;
    
    % Standard case
    for ii = 1:3
        t1 = (obj.minCorner(ii) - pos1(ii)) / d(ii);
        t2 = (obj.maxCorner(ii) - pos2(ii)) / d(ii);
        tmin = max(tmin, min(t1, t2));
        tmax = min(tmax, max(t1, t2));
        if tmin > tmax
            c = false;
            return;
        end
    end
    
    c = (tmax >= 0) && (tmin <= 1);
    end