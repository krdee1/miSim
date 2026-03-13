classdef miSim
    % multiagent interconnection simulation

    % Simulation parameters
    properties (SetAccess = public, GetAccess = public)
        timestep = NaN; % delta time interval for simulation iterations
        timestepIndex = NaN; % index of the current timestep (useful for time-indexed arrays)
        maxIter = NaN; % maximum number of simulation iterations
        domain;
        objective;
        obstacles; % geometries that define obstacles within the domain
        agents; % agents that move within the domain
        adjacency = false(0, 0); % Adjacency matrix representing communications network graph
        constraintAdjacencyMatrix = false(0, 0); % Adjacency matrix representing desired lesser neighbor connections
        partitioning = zeros(0, 0);
        perf; % sensor performance timeseries array
        performance = 0; % simulation performance timeseries vector
        barrierGain = NaN; % CBF gain parameter
        barrierExponent = NaN; % CBF exponent parameter
        minAlt = 0; % minimum allowable altitude (m)
        useDoubleIntegrator = false; % false = single-integrator, true = double-integrator dynamics
        dampingCoeff = 2.0; % velocity-proportional damping for double-integrator mode
        useFixedTopology = false; % false = lesser neighbor (dynamic), true = fixed initial topology
        artifactName = "";
        f; % main plotting tiled layout figure
        fPerf; % performance plot figure
        % Indicies for various plot types in the main tiled layout figure
        spatialPlotIndices = [6, 4, 3, 2];
        numBarriers = 0; % Number of barrier functions needed
        barriers = []; % log barrier function values at each timestep for analysis
    end

    properties (Access = private)
        % Sim
        t = NaN; % current sim time
        times;
        partitioningTimes;

        % Plot objects
        makePlots = true; % enable/disable simulation plotting (performance implications)
        makeVideo = true; % enable/disable VideoWriter (performance implications)
        connectionsPlot; % objects for lines connecting agents in spatial plots
        graphPlot; % objects for abstract network graph plot
        partitionPlot; % objects for partition plot
        performancePlot; % objects for sensor performance plot

        posHist; % data for trail plot
        velHist; % velocity history (double-integrator mode)
        trailPlot; % objects for agent trail plot

        % Indicies for various plot types in the main tiled layout figure
        objectivePlotIndices = [6, 4];
        networkGraphIndex = 5;
        partitionGraphIndex = 1;

        % CBF plotting
        h; % h function values
        hf; % h function plotting figure
        caPlot; % objects for collision avoidance h function plot
        obsPlot; % objects for obstacle h function plot
        domPlot; % objects for domain h function plot
    end

    methods (Access = public)
        function obj = miSim()
            arguments (Output)
                obj (1, 1) miSim
            end
            obj.domain = rectangularPrism;
            obj.objective = sensingObjective;
            obj.obstacles = {rectangularPrism};
            obj.agents = {agent};
        end
        [obj] = initialize(obj, domain, agents, barrierGain, barrierExponent, minAlt, timestep, maxIter, obstacles, makePlots, makeVideo);
        [obj] = initializeFromCsv(obj, csvPath);
        [obj] = run(obj);
        [obj] = lesserNeighbor(obj);
        [obj] = constrainMotion(obj);
        [obj] = partition(obj);
        [obj] = updateAdjacency(obj);
        [obj] = plot(obj);
        [obj] = plotConnections(obj);
        [obj] = plotPartitions(obj);
        [obj] = plotGraph(obj);
        [obj] = plotTrails(obj);
        [obj] = plotH(obj);
        [obj] = updatePlots(obj);
        [obj] = teardown(obj);
        writeInits(obj);
        validate(obj);
    end
    methods (Access = private)
        [v] = setupVideoWriter(obj);
    end
end
