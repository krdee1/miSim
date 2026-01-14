function validate(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
    end

    %% Communications Network Validators
    if max(conncomp(graph(obj.adjacency))) ~= 1
        warning("Network is not connected");
    end

    if any(obj.adjacency - obj.constraintAdjacencyMatrix < 0, 'all')
        warning("Eliminated network connections that were necessary");
    end

    %% Obstacle Validators
    AO_collisions = cellfun(@(a) cellfun(@(o) o.contains(a.pos), obj.obstacles), obj.agents, 'UniformOutput', false);
    AO_collisions = vertcat(AO_collisions{:});
    if any(AO_collisions)
        [idx, idy] = find(AO_collisions);
        for ii = 1:size(idx, 1)
            error("Agent(s) %d colliding with obstacle(s) %d", idx(ii), idy(ii));
        end
    end

end
