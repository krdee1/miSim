function obj = updateAdjacency(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    % Initialize assuming only self-connections
    A = true(size(obj.agents, 1));

    % Check lower triangle off-diagonal connections
    for ii = 2:size(A, 1)
        for jj = 1:(ii - 1)
            % Check that agents are not out of range
            if norm(obj.agents{ii}.pos - obj.agents{jj}.pos) > min([obj.agents{ii}.commsGeometry.radius, obj.agents{jj}.commsGeometry.radius])
                A(ii, jj) = false; % comm range violation
                continue;
            end
        end
    end

    obj.adjacency = A & A';
end