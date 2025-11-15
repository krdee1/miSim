function obj = updateAdjacency(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end

    % Initialize assuming only self-connections
    A = logical(eye(size(obj.agents, 1)));

    % Check lower triangle off-diagonal connections
    for ii = 2:size(A, 1)
        for jj = 1:(ii - 1)
            if norm(obj.agents{ii}.pos - obj.agents{jj}.pos) <= min([obj.agents{ii}.comRange, obj.agents{jj}.comRange])
                % Make sure that obstacles don't obstruct the line
                % of sight, breaking the connection
                for kk = 1:size(obj.obstacles, 1)
                    if ~obj.obstacles{kk}.containsLine(obj.agents{ii}.pos, obj.agents{jj}.pos)
                        A(ii, jj) = true;
                    end
                end
                % need extra handling for cases with no obstacles
                if isempty(obj.obstacles)
                    A(ii, jj) = true;
                end
            end
        end
    end

    obj.adjacency = A | A';
end