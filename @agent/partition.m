function [partitioning] = partition(obj, agents, objective)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'agent')};
        agents (:, 1) {mustBeA(agents, 'cell')};
        objective (1, 1) {mustBeA(objective, 'sensingObjective')};
    end
    arguments (Output)
        partitioning (:, :) double;
    end
    
    % Assess sensing performance of each agent at each sample point
    % in the domain
    agentPerformances = cellfun(@(x) reshape(x.sensorModel.sensorPerformance(x.pos, x.pan, x.tilt, [objective.X(:), objective.Y(:), zeros(size(objective.X(:)))]), size(objective.X)), agents, 'UniformOutput', false);
    agentPerformances{end + 1} = objective.sensorPerformanceMinimum * ones(size(agentPerformances{end})); % add additional layer to represent the threshold that has to be cleared for assignment to any partiton
    agentPerformances = cat(3, agentPerformances{:});

    % Get highest performance value at each point
    [~, idx] = max(agentPerformances, [], 3);

    % Collect agent indices in the same way as performance
    indices = 1:size(agents, 1);
    agentInds = squeeze(tensorprod(indices, ones(size(objective.X))));
    if size(agentInds, 1) ~= size(agents, 1)
        agentInds = reshape(agentInds, [size(agents, 1), size(agentInds)]); % needed for cases with 1 agent where prior squeeze is too agressive
    end
    agentInds = num2cell(agentInds, 2:3);
    agentInds = cellfun(@(x) squeeze(x), agentInds, 'UniformOutput', false);
    agentInds{end + 1} = zeros(size(agentInds{end})); % index for no assignment
    agentInds = cat(3, agentInds{:});

    % Use highest performing agent's index to form partitions
    [m, n, ~] = size(agentInds);
    [jj, kk] = ndgrid(1:m, 1:n);
    partitioning = agentInds(sub2ind(size(agentInds), jj, kk, idx));
end