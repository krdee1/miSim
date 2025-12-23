function obj = initialize(obj, domain, objective, agents, minAlt, timestep, partitoningFreq, maxIter, obstacles, makeVideo)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        domain (1, 1) {mustBeGeometry};
        objective (1, 1) {mustBeA(objective, 'sensingObjective')};
        agents (:, 1) cell;
        minAlt (1, 1) double = 1;
        timestep (:, 1) double = 0.05;
        partitoningFreq (:, 1) double = 0.25
        maxIter (:, 1) double = 1000;
        obstacles (:, 1) cell {mustBeGeometry} = cell(0, 1);
        makeVideo (1, 1) logical = true;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end

    % enable/disable video writer
    obj.makeVideo = makeVideo;

    % Define simulation time parameters
    obj.timestep = timestep;
    obj.maxIter = maxIter - 1;

    % Define domain
    obj.domain = domain;
    obj.partitioningFreq = partitoningFreq;

    % Add geometries representing obstacles within the domain
    obj.obstacles = obstacles;

    % Add an additional obstacle spanning the domain's footprint to 
    % represent the minimum allowable altitude
    obj.minAlt = minAlt;
    obj.obstacles{end + 1, 1} = rectangularPrism;
    obj.obstacles{end, 1} = obj.obstacles{end, 1}.initialize([obj.domain.minCorner; obj.domain.maxCorner(1:2), obj.minAlt], "OBSTACLE", "Minimum Altitude Domain Constraint");

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