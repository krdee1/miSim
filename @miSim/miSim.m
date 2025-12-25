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
        sensorPerformanceMinimum = 1e-6; % minimum sensor performance to allow assignment of a point in the domain to a partition
        partitioning = NaN;
        performance = 0; % cumulative sensor performance
        barrierGain = 100; % collision avoidance parameter
        minAlt = 1; % minimum allowed altitude constraint

        fPerf; % performance plot figure
    end

    properties (Access = private)
        % Sim
        t = NaN; % current sim time
        perf; % sensor performance timeseries array
        times;
        partitioningTimes;

        % Plot objects
        makePlots = true; % enable/disable simulation plotting (performance implications)
        makeVideo = true; % enable/disable VideoWriter (performance implications)
        f; % main plotting tiled layout figure
        connectionsPlot; % objects for lines connecting agents in spatial plots
        graphPlot; % objects for abstract network graph plot
        partitionPlot; % objects for partition plot

        performancePlot; % objects for sensor performance plot

        % Indicies for various plot types in the main tiled layout figure
        spatialPlotIndices = [6, 4, 3, 2];
        objectivePlotIndices = [6, 4];
        networkGraphIndex = 5;
        partitionGraphIndex = 1;
    end

    methods (Access = public)
        [obj] = initialize(obj, domain, objective, agents, timestep, partitoningFreq, maxIter, obstacles);
        [obj] = run(obj);
        [obj] = constrainMotion(obj);
        [obj] = partition(obj);
        [obj] = updateAdjacency(obj);
        [obj] = plot(obj);
        [obj] = plotConnections(obj);
        [obj] = plotPartitions(obj);
        [obj] = plotGraph(obj);
        [obj] = updatePlots(obj, updatePartitions);
    end
    methods (Access = private)
        [v] = setupVideoWriter(obj);
    end
end
