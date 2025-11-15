function c = geometryIntersects(g1, g2)
    c = false;
    % Check if g2 contains g1
    for jj = 1:size(g1.edges, 1)
        if g2.containsLine(g1.vertices(g1.edges(jj, 1), 1:3), g1.vertices(g1.edges(jj, 2), 1:3))
            c = true;
            return;
        end
    end
    % Check if g1 contains g2
    for jj = 1:size(g2.edges, 1)
        if g1.containsLine(g2.vertices(g2.edges(jj, 1), 1:3), g2.vertices(g2.edges(jj, 2), 1:3))
            c = true;
            return;
        end
    end
end