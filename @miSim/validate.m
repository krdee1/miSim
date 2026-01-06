function validate(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
    end

    if max(conncomp(graph(obj.adjacency))) ~= 1
        warning("Network is not connected");
    end

end
