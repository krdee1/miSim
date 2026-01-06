function obj = lesserNeighbor(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end

    % initialize solution with self-connections only
    constraintAdjacencyMatrix = logical(eye(size(obj.agents, 1)));

    for ii = 1:size(obj.agents, 1)
        % Find lesser neighbors of each agent
        % Lesser neighbors of ii are jj < ii in range of ii
        lesserNeighbors = [];
        for jj = 1:(ii - 1)
            if obj.adjacency(ii, jj)
                lesserNeighbors = [lesserNeighbors, jj];
            end
        end
        obj.agents{ii}.lesserNeighbors = lesserNeighbors;
        
        % Early exit for isolated agents
        if isempty(obj.agents{ii}.lesserNeighbors)
            continue
        end

        % Focus on subgraph defined by lesser neighbors
        subgraphAdjacency = obj.adjacency(obj.agents{ii}.lesserNeighbors, obj.agents{ii}.lesserNeighbors);

        % Find connected components in each agent's subgraph
        % TODO: rewrite this using matlab "conncomp" function?
        visited = false(size(subgraphAdjacency, 1), 1);
        components = {};
        for jj = 1:size(subgraphAdjacency, 1)
            if ~visited(jj)
                reachable = bfs(subgraphAdjacency, jj);
                visited(reachable) = true;
                components{end+1} = obj.agents{ii}.lesserNeighbors(reachable);
            end
        end

        % Connect to the greatest index in each connected component in the
        % lesser neighborhood of this agent
        for jj = 1:size(components, 2)
            constraintAdjacencyMatrix(ii, max(components{jj})) = true;
            constraintAdjacencyMatrix(max(components{jj}), ii) = true;
        end
    end
    obj.constraintAdjacencyMatrix = constraintAdjacencyMatrix | constraintAdjacencyMatrix';
end

function cComp = bfs(subgraphAdjacency, startIdx)
    n = size(subgraphAdjacency, 1);
    visited = false(1, n);
    queue = startIdx;
    cComp = startIdx;
    visited(startIdx) = true;
    
    while ~isempty(queue)
        current = queue(1);
        queue(1) = [];
        
        % Find all neighbors of current node in the subgraph
        neighbors = find(subgraphAdjacency(current, :));
        
        for neighbor = neighbors
            if ~visited(neighbor)
                visited(neighbor) = true;
                cComp = [cComp, neighbor];
                queue = [queue, neighbor];
            end
        end
    end
    cComp = sort(cComp);
end