function obj = partition(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end

    % Assess sensing performance of each agent at each sample point
    % in the domain
    agentPerformances = cellfun(@(x) reshape(x.sensorModel.sensorPerformance(x.pos, x.pan, x.tilt, [obj.objective.X(:), obj.objective.Y(:), zeros(size(obj.objective.X(:)))]), size(obj.objective.X)), obj.agents, 'UniformOutput', false);
    agentPerformances{end + 1} = obj.sensorPerformanceMinimum * ones(size(agentPerformances{end})); % add additional layer to represent the threshold that has to be cleared for assignment to any partiton
    agentPerformances = cat(3, agentPerformances{:});
    
    % Get highest performance value at each point
    [~, idx] = max(agentPerformances, [], 3);

    % Collect agent indices in the same way as performance
    agentInds = cellfun(@(x) x.index * ones(size(obj.objective.X)), obj.agents, 'UniformOutput', false);
    agentInds{end + 1} = zeros(size(agentInds{end})); % index for no assignment
    agentInds = cat(3, agentInds{:});

    % Get highest performing agent's index
    [m,n,~] = size(agentInds);
    [i,j] = ndgrid(1:m, 1:n);
    obj.partitioning = agentInds(sub2ind(size(agentInds), i, j, idx));

    % Get individual agent sensor performance
    for ii = 1:size(obj.agents, 1)
        obj.agents{ii}.performance = sum(agentPerformances(sub2ind(size(agentInds), i, j, idx)), 'all');
    end

    % Current total performance
    sum(arrayfun(@(x) x.performance, [obj.agents{:}]))
    obj.performance = sum(max(agentPerformances(:, :, 1:(end - 1)), [], 3), 'all'); % do not count final "non-assignment" layer in computing cumulative performance
end