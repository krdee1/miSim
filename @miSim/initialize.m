function [obj, f] = initialize(obj, domain, objective, agents, timestep, partitoningFreq, maxIter, obstacles)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        domain (1, 1) {mustBeGeometry};
        objective (1, 1) {mustBeA(objective, 'sensingObjective')};
        agents (:, 1) cell;
        timestep (:, 1) double = 0.05;
        partitoningFreq (:, 1) double = 0.25
        maxIter (:, 1) double = 1000;
        obstacles (:, 1) cell {mustBeGeometry} = cell(0, 1);
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
    end

    % Define simulation time parameters
    obj.timestep = timestep;
    obj.maxIter = maxIter;

    % Define domain
    obj.domain = domain;
    obj.partitioningFreq = partitoningFreq;

    % Add geometries representing obstacles within the domain
    obj.obstacles = obstacles;

    % Define objective
    obj.objective = objective;

    % Define agents
    obj.agents = agents;

    % Compute adjacency matrix
    obj = obj.updateAdjacency();

    % Create initial partitioning
    obj = obj.partition();

    % Set up plots showing initialized state
    [obj, f] = obj.plot();
end