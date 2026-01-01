function obj = updateAdjacency(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end

    % Initialize assuming only self-connections
    A = true(size(obj.agents, 1));

    % Check lower triangle off-diagonal connections
    for ii = 2:size(A, 1)
        for jj = 1:(ii - 1)
            % Check that agents are not out of range
            if norm(obj.agents{ii}.pos - obj.agents{jj}.pos) > min([obj.agents{ii}.comRange, obj.agents{jj}.comRange]);
                A(ii, jj) = false; % comm range violation
                continue;
            end

            % % Check that agents do not have their line of sight obstructed
            % for kk = 1:size(obj.obstacles, 1)
            %     if obj.obstacles{kk}.containsLine(obj.agents{jj}.pos, obj.agents{ii}.pos)
            %         A(ii, jj) = false;
            %     end
            % end
        end
    end

    obj.adjacency = A & A';

    if any(obj.adjacency - obj.constraintAdjacencyMatrix < 0, 'all')
        warning("Eliminated network connections that were necessary");
        keyboard
    end
end