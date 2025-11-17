function obj = initialize(obj, domain, objective, agents, timestep, partitoningFreq, maxIter, obstacles)
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
    end

    % Define simulation time parameters
    obj.timestep = timestep;
    obj.maxIter = maxIter - 1;

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

    % Set up times to iterate over
    obj.times = linspace(0, obj.timestep * obj.maxIter, obj.maxIter+1)';
    obj.partitioningTimes = obj.times(obj.partitioningFreq:obj.partitioningFreq:size(obj.times, 1));

    % Prepare performance data store (at t = 0, all have 0 performance)
    obj.fPerf = figure;
    obj.perf = [zeros(size(obj.agents, 1) + 1, 1), NaN(size(obj.agents, 1) + 1, size(obj.partitioningTimes, 1) - 1)];

    % Create initial partitioning
    obj = obj.partition();

    % Set up plots showing initialized state
    obj = obj.plot();
end