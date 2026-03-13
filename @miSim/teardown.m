function obj = teardown(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    % Close plots
    close(obj.hf);
    close(obj.fPerf);
    close(obj.f);

    % Log results into matfile
    histPath = fullfile(matlab.project.rootProject().RootFolder, "sandbox", strcat(obj.artifactName, "_miSimHist.mat"));
    out = struct("agent", repmat(struct("pos", [], "perf", [], "sensor", struct("alphaDist", [], "betaDist", [], "alphaTilt", [], "betaTilt", []), "collisionRadius", [], "commsRadius", []), size(obj.agents)), "perf", [], "barriers", []);

    out.perf = obj.performance(1:(end - 1));
    out.barriers = [zeros(size(obj.barriers(1:end, 1), 1), 1), obj.barriers(1:end, 1:(end - 1))];
    for ii = 1:size(obj.agents, 1)
        out.agent(ii).pos = squeeze(obj.posHist(ii, 1:(end - 1), 1:3));
        out.agent(ii).perf = obj.agents{ii}.performance(1:(end - 2));
        out.agent(ii).sensor.alphaDist = obj.agents{ii}.sensorModel.alphaDist;
        out.agent(ii).sensor.betaDist = obj.agents{ii}.sensorModel.betaDist;
        out.agent(ii).sensor.alphaTilt = obj.agents{ii}.sensorModel.alphaTilt;
        out.agent(ii).sensor.betaTilt = obj.agents{ii}.sensorModel.betaTilt;
        out.agent(ii).collisionRadius = obj.agents{ii}.collisionGeometry.radius;
        out.agent(ii).commsRadius = obj.agents{ii}.commsGeometry.radius;
    end

    save(histPath, "out");

    % reset parameters
    obj.timestep = NaN;
    obj.timestepIndex = NaN;
    obj.maxIter = NaN;
    obj.domain = rectangularPrism;
    obj.objective = sensingObjective;
    obj.obstacles = cell(0, 1);
    obj.agents = cell(0, 1);
    obj.adjacency = NaN;
    obj.constraintAdjacencyMatrix = NaN;
    obj.partitioning = NaN;
    obj.performance = 0;
    obj.barrierGain = NaN;
    obj.barrierExponent = NaN;
    obj.artifactName = "";

end