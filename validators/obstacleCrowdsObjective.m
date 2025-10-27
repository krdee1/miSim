function c = obstacleCrowdsObjective(objective, obstacle, protectedRange)
    arguments (Input)
        objective (1, 1) {mustBeA(objective, 'sensingObjective')};
        obstacle (1, 1) {mustBeGeometry}; % this could be expanded to handle n obstacles in 1 call
        protectedRange (1, 1) double;
    end
    arguments (Output)
        c (1, 1) logical;
    end
    c = norm(obstacle.distance([objective.groundPos, obstacle.center(3)])) <= protectedRange;
end