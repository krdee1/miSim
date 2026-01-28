function c = obstacleCoversObjective(objective, obstacle)
    arguments (Input)
        objective (1, 1) {mustBeA(objective, "sensingObjective")};
        obstacle (1, 1) {mustBeGeometry}; % this could be expanded to handle n obstacles in 1 call
    end
    arguments (Output)
        c (1, 1) logical;
    end

    % Check if the obstacle contains the objective's ground position if the
    % ground position were raised to the obstacle's center's height
    c = obstacle.contains([objective.groundPos, obstacle.center(3)]);
end