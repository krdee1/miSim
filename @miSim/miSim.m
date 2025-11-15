classdef miSim
    % multiagent interconnection simulation

    % Simulation parameters
    properties (SetAccess = private, GetAccess = public)
        timestep = NaN; % delta time interval for simulation iterations
        partitioningFreq = NaN; % number of simulation timesteps at which the partitioning routine is re-run
        maxIter = NaN; % maximum number of simulation iterations
        domain = rectangularPrism;
        objective = sensingObjective;
        obstacles = cell(0, 1); % geometries that define obstacles within the domain
        agents = cell(0, 1); % agents that move within the domain
        adjacency = NaN; % Adjacency matrix representing communications network graph
        partitioning = NaN;
    end

    properties (Access = private)
        % Plot objects
        connectionsPlot; % objects for lines connecting agents in spatial plots
        graphPlot; % objects for abstract network graph plot
        partitionPlot; % objects for partition plot

        % Indicies for various plot types in the main tiled layout figure
        spatialPlotIndices = [6, 4, 3, 2];
        objectivePlotIndices = [6, 4];
        networkGraphIndex = 5;
        partitionGraphIndex = 1;
    end

    methods (Access = public)
        [obj, f] = initialize(obj, domain, objective, agents, timestep, partitoningFreq, maxIter, obstacles);
        [obj, f] = run(obj, f);
        [obj]    = partition(obj);
        [obj]    = updateAdjacency(obj);
        [obj, f] = plot(obj);
        [obj, f] = plotConnections(obj, ind, f);
        [obj, f] = plotPartitions(obj, ind, f);
        [obj, f] = plotGraph(obj, ind, f);
        [obj, f] = updatePlots(obj, f, updatePartitions);
    end
    methods (Access = private)
        [v] = setupVideoWriter(obj);
    end
end