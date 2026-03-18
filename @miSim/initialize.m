function [obj] = initialize(obj, domain, agents, barrierGain, barrierExponent, minAlt, timestep, maxIter, obstacles, makePlots, makeVideo, useDoubleIntegrator, dampingCoeff, useFixedTopology)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
        domain (1, 1) {mustBeGeometry};
        agents (:, 1) cell;
        barrierGain (1, 1) double = 100;
        barrierExponent (1, 1) double = 3;
        minAlt (1, 1) double = 1;
        timestep (:, 1) double = 0.05;
        maxIter (:, 1) double = 1000;
        obstacles (:, 1) cell {mustBeGeometry} = cell(0, 1);
        makePlots(1, 1) logical = true;
        makeVideo (1, 1) logical = true;
        useDoubleIntegrator (1, 1) logical = false;
        dampingCoeff (1, 1) double = 2.0;
        useFixedTopology (1, 1) logical = false;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    % enable/disable plotting and video writer
    obj.makePlots = makePlots;
    if ~obj.makePlots
        if makeVideo
            if coder.target('MATLAB')
                warning("makeVideo set to true, but makePlots set to false. Setting makeVideo to false.");
            end
            makeVideo = false;
        end
    end
    obj.makeVideo = makeVideo;

    % Generate artifact(s) name
    if coder.target('MATLAB')
        obj.artifactName = strcat(string(datetime("now"), "yyyy_MM_dd_HH_mm_ss"));
    else
        obj.artifactName = ""; % Generate no artifacts from simulation in codegen
    end

    % Define simulation time parameters
    obj.timestep = timestep;
    obj.timestepIndex = 0;
    obj.maxIter = maxIter - 1;

    % Define domain
    obj.domain = domain;

    % Add geometries representing obstacles within the domain, pre-allocating
    % one extra slot for the minimum altitude floor obstacle if needed
    numInputObs = size(obstacles, 1);
    if minAlt > 0
        obj.obstacles = repmat({rectangularPrism}, numInputObs + 1, 1);
    else
        obj.obstacles = repmat({rectangularPrism}, numInputObs, 1);
    end
    for kk = 1:numInputObs
        obj.obstacles{kk} = obstacles{kk};
    end

    % Add an additional obstacle spanning the domain's footprint to
    % represent the minimum allowable altitude
    if minAlt > 0
        minAltObstacle = rectangularPrism;
        minAltObstacle = minAltObstacle.initialize([obj.domain.minCorner; obj.domain.maxCorner(1:2), minAlt], "OBSTACLE", "Minimum Altitude Domain Constraint");
        obj.obstacles{numInputObs + 1} = minAltObstacle;
    end

    % Define agents
    obj.agents = agents;
    obj.constraintAdjacencyMatrix = logical(eye(size(agents, 1)));

    % Set labels for agents and collision geometries in cases where they
    % were not provieded at the time of their initialization
    for ii = 1:size(obj.agents, 1)
        % Agent
        if isempty(char(obj.agents{ii}.label))
            obj.agents{ii}.label = sprintf("Agent %d", int8(ii));
        end

        % Collision geometry
        if isempty(char(obj.agents{ii}.collisionGeometry.label))
            obj.agents{ii}.collisionGeometry.label = sprintf("Agent %d Collision Geometry", int8(ii));
        end
    end

    % Set CBF parameters
    obj.barrierGain = barrierGain;
    obj.barrierExponent = barrierExponent;
    obj.minAlt = minAlt;

    % Set dynamics model
    obj.useDoubleIntegrator = useDoubleIntegrator;
    obj.dampingCoeff = dampingCoeff;
    obj.useFixedTopology = useFixedTopology;

    % Compute adjacency matrix and network topology
    obj = obj.updateAdjacency();
    if obj.useFixedTopology
        obj.constraintAdjacencyMatrix = obj.adjacency;
    else
        obj = obj.lesserNeighbor();
    end

    % Set up times to iterate over
    obj.times = linspace(0, obj.timestep * obj.maxIter, obj.maxIter+1)';

    if coder.target('MATLAB')
        % Prepare performance data store (at t = 0, all have 0 performance)
        obj.perf = [zeros(size(obj.agents, 1) + 1, 1), NaN(size(obj.agents, 1) + 1, size(obj.partitioningTimes, 1) - 1)];

        % Prepare h function data store
        obj.h = NaN(size(obj.agents, 1) * (size(obj.agents, 1) - 1) / 2 + size(obj.agents, 1) * size(obj.obstacles, 1) + 6, size(obj.times, 1));
    end

    % Create initial partitioning
    obj.partitioning = obj.agents{1}.partition(obj.agents, obj.domain.objective);

    % Determine number of barrier functions that will be necessary
    if size(obj.agents, 1) < 2
        nAAPairs = 0;
    else
        nAAPairs = nchoosek(size(obj.agents, 1), 2); % unique agent/agent pairs
    end
    nAOPairs = size(obj.agents, 1) * size(obj.obstacles, 1); % unique agent/obstacle pairs
    nADPairs = size(obj.agents, 1) * 6; % agents x (4 walls + 1 floor + 1 ceiling)
    nLNAPairs = sum(triu(obj.constraintAdjacencyMatrix, 1), "all");
    obj.numBarriers = nAAPairs + nAOPairs + nADPairs + nLNAPairs;

    if coder.target('MATLAB')
        % Initialize variable that will store agent positions for trail plots
        obj.posHist = NaN(size(obj.agents, 1), obj.maxIter + 1, 3);
        obj.posHist(1:size(obj.agents, 1), 1, 1:3) = reshape(cell2mat(cellfun(@(x) x.pos, obj.agents, "UniformOutput", false)), size(obj.agents, 1), 1, 3);

        % Initialize velocity history (zeros at t=0, all agents start at rest)
        obj.velHist = zeros(size(obj.agents, 1), obj.maxIter + 1, 3);

        % Initialize variable that will store barrier function values per timestep for analysis purposes
        obj.barriers = NaN(obj.numBarriers, size(obj.times, 1));

        % Initialize constraint adjacency history (nAgents x nAgents x nTimesteps)
        nAgents = size(obj.agents, 1);
        obj.constraintAdjacencyHist = false(nAgents, nAgents, size(obj.times, 1));
        obj.constraintAdjacencyHist(:, :, 1) = obj.constraintAdjacencyMatrix;

        % Set up plots showing initialized state
        obj = obj.plot();

        % Run validations
        obj.validate();
    end
end