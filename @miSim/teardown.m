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