function validate(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
    end

    %% Communications Network Validators
    if max(conncomp(graph(obj.adjacency))) ~= 1
        warning("Network is not connected");
    end

    if any(obj.adjacency - obj.constraintAdjacencyMatrix < 0, "all")
        warning("Eliminated network connections that were necessary");
    end

    %% Obstacle Validators
    % Agent-Obstacle Collision Detection
    for jj = 1:size(obj.obstacles, 1)
        for kk = 1:size(obj.agents, 1)
            P = min(max(obj.agents{kk}.pos, obj.obstacles{jj}.minCorner), obj.obstacles{jj}.maxCorner);
            d = obj.agents{kk}.pos - P;
            if dot(d, d) <= obj.agents{kk}.collisionGeometry.radius^2
                error("%s colliding with %s", obj.agents{kk}.label, obj.obstacles{jj}.label); % this will cause quadprog to fail
            end
        end
    end

end
