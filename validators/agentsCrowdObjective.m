function c = agentsCrowdObjective(objective, positions, protectedRange)
    arguments (Input)
        objective (1, 1) {mustBeA(objective, 'sensingObjective')};
        positions (:, 3) double; % this could be expanded to handle n obstacles in 1 call
        protectedRange (1, 1) double;
    end
    arguments (Output)
        c (:, 1) logical;
    end
    c = vecnorm(positions(:, 1:2) - objective.groundPos, 2, 2) <= protectedRange;
end