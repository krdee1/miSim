function [values, positions] = sense(obj, agent, sensingObjective, domain, partitioning)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'sigmoidSensor')};
        agent (1, 1) {mustBeA(agent, 'agent')};
        sensingObjective (1, 1) {mustBeA(sensingObjective, 'sensingObjective')};
        domain (1, 1) {mustBeGeometry};
        partitioning (:, :) double;
    end
    arguments (Output)
        values (:, 1) double;
        positions (:, 3) double;
    end

    % Find positions for this agent's assigned partition in the domain
    idx = partitioning == agent.index;
    positions = [sensingObjective.X(idx), sensingObjective.Y(idx), zeros(size(sensingObjective.X(idx)))];

    % Evaluate objective function at every point in this agent's
    % assigned partiton
    values = sensingObjective.values(idx);
end