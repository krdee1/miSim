function obj = lesserNeighbor(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end

    % Check possible connections from adjacency matrix
    % Choose connections which fully connect network by selecting maximum
    % indices according to the previous columns (or rows) of the new
    % constraint adjacency matrix
    % Place that choice in the constraint adjacency matrix

    % Begin with all possible connections and trim down
    constraintAdjacencyMatrix = obj.adjacency;

    % Iterate over each agent (by increasing index)
    for ii = 1:size(obj.agents, 1)
        % Iterate over each agent of lesser index and see if a higher
        % indexed agent provides connectivity already
        for jj = 1:(ii - 1)
            for kk = 1:(jj - 1)
                constraintAdjacencyMatrix(ii, kk) = false;
                constraintAdjacencyMatrix(kk, ii) = false;
            end
        end
    end

    obj.constraintAdjacencyMatrix = constraintAdjacencyMatrix;
end