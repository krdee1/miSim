function obj = teardown(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    % % Close plots
    % close(obj.hf);
    % close(obj.fPerf);
    % close(obj.f);

    % Log results into matfile
    histPath = fullfile(matlab.project.rootProject().RootFolder, "sandbox", strcat(obj.artifactName, "_miSimHist.mat"));
    out = struct("agent", repmat(struct("pos", [], "vel", [], "perf", [], "sensor", struct(), "collisionRadius", [], "commsRadius", []), size(obj.agents)), "perf", [], "barriers", [], "useDoubleIntegrator", [], "dampingCoeff", [], "useFixedTopology", []);

    out.perf = obj.performance(1:(end - 1));
    out.barriers = [zeros(size(obj.barriers(1:end, 1), 1), 1), obj.barriers(1:end, 1:(end - 1))];
    out.dampingCoeff = obj.dampingCoeff;
    out.useDoubleIntegrator = obj.useDoubleIntegrator;
    out.useFixedTopology = obj.useFixedTopology;
    out.constraintAdjacency = obj.constraintAdjacencyHist(:, :, 1:(end - 1));

    % CALSN routing state at the final step (empty unless useRoutingTopology)
    out.useRoutingTopology = obj.useRoutingTopology;
    out.routingFlows = obj.routingFlows;
    out.routingPotentials = obj.routingPotentials;
    out.routingUnrouted = obj.routingUnrouted;
    for ii = 1:size(obj.agents, 1)
        out.agent(ii).pos = squeeze(obj.posHist(ii, 1:(end - 1), 1:3));
        out.agent(ii).vel = squeeze(obj.velHist(ii, 1:(end - 1), 1:3));
        out.agent(ii).perf = obj.agents{ii}.performance(1:(end - 2));
        % Sensor parameters depend on the sensor model; tilt/azimuth are the
        % final pointing (the initial pointing is in the _miSimInits file)
        sm = obj.agents{ii}.sensorModel;
        if isa(sm, "sigmoidSensor")
            out.agent(ii).sensor = struct("model", "sigmoidSensor", "alphaDist", sm.alphaDist, "betaDist", sm.betaDist, ...
                "alphaTilt", sm.alphaTilt, "betaTilt", sm.betaTilt, "tilt", sm.tilt, "azimuth", sm.azimuth);
        elseif isa(sm, "rfSensor")
            out.agent(ii).sensor = struct("model", "rfSensor", "P_TX", sm.P_TX, "T_0", sm.T_0, "BW", sm.BW, "f_c", sm.f_c, ...
                "G_RX_dBi", sm.G_RX_dBi, "beamwidthExponent", sm.beamwidthExponent, "lossExponent", sm.lossExponent, ...
                "tilt", sm.tilt, "azimuth", sm.azimuth);
        end
        out.agent(ii).collisionRadius = obj.agents{ii}.collisionGeometry.radius;
        out.agent(ii).commsRadius = obj.agents{ii}.commsGeometry.radius;
    end

    save(histPath, "out");

    % reset parameters
    obj.timestep = NaN;
    obj.timestepIndex = NaN;
    obj.maxIter = NaN;
    obj.domain = rectangularPrism;
    obj.obstacles = cell(0, 1);
    obj.agents = cell(0, 1);
    obj.adjacency = NaN;
    obj.constraintAdjacencyMatrix = NaN;
    obj.constraintAdjacencyHist = [];
    obj.partitioning = NaN;
    obj.performance = 0;
    obj.barrierGain = NaN;
    obj.barrierExponent = NaN;
    obj.useDoubleIntegrator = false;
    obj.dampingCoeff = 2.0;
    obj.useFixedTopology = false;
    obj.artifactName = "";

end