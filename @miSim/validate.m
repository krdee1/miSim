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

    %% 

end
