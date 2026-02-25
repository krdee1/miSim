function obj = lesserNeighbor(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    % initialize solution with self-connections only
    constraintAdjacencyMatrix = logical(eye(size(obj.agents, 1)));

    nAgents = size(obj.agents, 1);
    for ii = 1:nAgents
        % Find lesser neighbors of each agent
        % Lesser neighbors of ii are jj < ii in range of ii
        lnBuf = zeros(1, nAgents);
        lnCount = 0;
        for jj = 1:(ii - 1)
            if obj.adjacency(ii, jj)
                lnCount = lnCount + 1;
                lnBuf(lnCount) = jj;
            end
        end
        obj.agents{ii}.lesserNeighbors = lnBuf(1:lnCount);

        % Early exit for isolated agents
        if lnCount == 0
            continue
        end

        % Focus on subgraph defined by lesser neighbors
        subgraphAdjacency = obj.adjacency(obj.agents{ii}.lesserNeighbors, obj.agents{ii}.lesserNeighbors);

        % Find connected components; store only the max global index per
        % component (the only value needed below) to avoid a cell array
        visited = false(1, lnCount);
        maxInComponent = zeros(1, lnCount);
        numComponents = 0;
        for jj = 1:lnCount
            if ~visited(jj)
                reachable = bfs(subgraphAdjacency, jj);
                visited(reachable) = true;
                numComponents = numComponents + 1;
                maxInComponent(numComponents) = max(obj.agents{ii}.lesserNeighbors(reachable));
            end
        end

        % Connect to the greatest index in each connected component in the
        % lesser neighborhood of this agent
        for jj = 1:numComponents
            maxIdx = maxInComponent(jj);
            constraintAdjacencyMatrix(ii, maxIdx) = true;
            constraintAdjacencyMatrix(maxIdx, ii) = true;
        end
    end
    obj.constraintAdjacencyMatrix = constraintAdjacencyMatrix | constraintAdjacencyMatrix';
end

function cComp = bfs(subgraphAdjacency, startIdx)
    n = size(subgraphAdjacency, 1);
    visited = false(1, n);

    % Pre-allocated queue and component buffer with head/tail pointers
    % to avoid element deletion and dynamic array growth
    queue    = zeros(1, n);
    cCompBuf = zeros(1, n);
    qHead = 1;
    qTail = 2;
    queue(1)    = startIdx;
    cCompBuf(1) = startIdx;
    cSize = 1;
    visited(startIdx) = true;

    while qHead < qTail
        current = queue(qHead);
        qHead = qHead + 1;

        % Find all neighbors of current node in the subgraph
        neighbors = find(subgraphAdjacency(current, :));
        for kk = 1:numel(neighbors)
            neighbor = neighbors(kk);
            if ~visited(neighbor)
                visited(neighbor) = true;
                cCompBuf(cSize + 1) = neighbor;
                cSize = cSize + 1;
                queue(qTail) = neighbor;
                qTail = qTail + 1;
            end
        end
    end
    cComp = sort(cCompBuf(1:cSize));
end