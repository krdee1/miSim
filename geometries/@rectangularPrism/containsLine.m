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

    % endpoint contained (trivial case)
    if obj.contains(pos1) || obj.contains(pos2)
        c = true;
        return;
    end

    % parameterize the line segment to check for an intersection
    tMin = 0;
    tMax = 1;
    for ii = 1:3
        % line is parallel to geometry
        if abs(d(ii)) < 1e-12
            if pos1(ii) < obj.minCorner(ii) || pos1(ii) > obj.maxCorner(ii)
                c = false;
                return;
            end
        else
            t1 = (obj.minCorner(ii) - pos1(ii)) / d(ii);
            t2 = (obj.maxCorner(ii) - pos1(ii)) / d(ii);

            tLow  = min(t1, t2);
            tHigh = max(t1, t2);

            tMin = max(tMin, tLow);
            tMax = min(tMax, tHigh);

            if tMin > tMax
                c = false;
                return;
            end
        end
    end
    c = true;
    
    % if abs(d) < 1e-12
    %     % check if it happens to start or end inside or outside of
    %     % the geometry
    %     if obj.contains(pos1) || obj.contains(pos2)
    %         c = true;
    %     else
    %         c = false;
    %     end
    %     return;
    % end
    % 
    % tMin = -inf;        
    % tMax = inf;
    % 
    % % Standard case
    % for ii = 1:3
    %     t1 = (obj.minCorner(ii) - pos1(ii)) / d(ii);
    %     t2 = (obj.maxCorner(ii) - pos2(ii)) / d(ii);
    %     tMin = max(tMin, min(t1, t2));
    %     tMax = min(tMax, max(t1, t2));
    %     if tMin > tMax
    %         c = false;
    %         return;
    %     end
    % end
    % 
    % c = (tMax >= 0) && (tMin <= 1);
end