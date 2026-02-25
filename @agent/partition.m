function [partitioning] = partition(obj, agents, objective)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "agent")};
        agents (:, 1) {mustBeA(agents, "cell")};
        objective (1, 1) {mustBeA(objective, "sensingObjective")};
    end
    arguments (Output)
        partitioning (:, :) double;
    end

    nAgents = size(agents, 1);
    gridM   = size(objective.X, 1);
    gridN   = size(objective.X, 2);
    nPoints = gridM * gridN;

    % Assess sensing performance of each agent at each sample point.
    % agentPerf is (nPoints x nAgents+1): the extra column is the
    % minimum threshold that must be exceeded for any assignment.
    agentPerf = zeros(nPoints, nAgents + 1);
    for aa = 1:nAgents
        p = agents{aa}.sensorModel.sensorPerformance(agents{aa}.pos, ...
            [objective.X(:), objective.Y(:), zeros(nPoints, 1)]);
        agentPerf(:, aa) = p(:);
    end
    agentPerf(:, nAgents + 1) = objective.sensorPerformanceMinimum;

    % Find which agent has highest performance at each point.
    % If the threshold column wins (idx == nAgents+1) the point is unassigned (0).
    [~, idx] = max(agentPerf, [], 2);

    assignedAgent = zeros(nPoints, 1);
    for pp = 1:nPoints
        if idx(pp) <= nAgents
            assignedAgent(pp) = idx(pp);
        end
    end

    partitioning = reshape(assignedAgent, gridM, gridN);
end
