function writeInits(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
    end

    % User-supplied obstacles only: initialize() appends a floor obstacle at
    % the end when minAlt > 0, so exclude it here to avoid double-counting on
    % reconstruction (initializeFromInits re-adds the floor via minAlt).
    numInputObs = size(obj.obstacles, 1) - (obj.minAlt > 0);
    userObstacles = obj.obstacles(1:numInputObs);

    % Collect agent parameters
    collisionRadii = cellfun(@(x) x.collisionGeometry.radius, obj.agents);
    alphaDist = cellfun(@(x) x.sensorModel.alphaDist, obj.agents);
    betaDist = cellfun(@(x) x.sensorModel.betaDist, obj.agents);
    alphaTilt = cellfun(@(x) x.sensorModel.alphaTilt, obj.agents);
    betaTilt = cellfun(@(x) x.sensorModel.betaTilt, obj.agents);
    comRanges = cellfun(@(x) x.commsGeometry.radius, obj.agents);
    initialStepSize = cellfun(@(x) x.initialStepSize, obj.agents);
    pos = cell2mat(cellfun(@(x) x.pos, obj.agents, 'UniformOutput', false));
    obsMinCorners = cell2mat(cellfun(@(x) x.minCorner, userObstacles, 'UniformOutput', false));
    obsMaxCorners = cell2mat(cellfun(@(x) x.maxCorner, userObstacles, 'UniformOutput', false));

    % Combine with simulation parameters
    inits = struct("timestep", obj.timestep, "maxIter", obj.maxIter + 1, "minAlt", obj.minAlt, ...
                    "discretizationStep", obj.domain.objective.discretizationStep, "protectedRange", obj.domain.objective.protectedRange, ...
                    "sensorPerformanceMinimum", obj.domain.objective.sensorPerformanceMinimum, "initialStepSize", initialStepSize, ...
                    "barrierGain", obj.barrierGain, "barrierExponent", obj.barrierExponent, "numObstacles", numInputObs, ...
                    "numAgents", size(obj.agents, 1), "collisionRadius", collisionRadii, "comRange", comRanges, ...
                    "useDoubleIntegrator", obj.useDoubleIntegrator, "dampingCoeff", obj.dampingCoeff, "useFixedTopology", obj.useFixedTopology, ...
                    "alphaDist", alphaDist, "betaDist", betaDist, "alphaTilt", alphaTilt, "betaTilt", betaTilt, ...
                    ... % ^^^ PARAMETERS ^^^ | vvv STATES vvv
                    "pos", pos, "objectivePos", obj.domain.objective.groundPos, "objectiveSigma", obj.domain.objective.objectiveSigma, ...
                    "domainMin", obj.domain.minCorner, "domainMax", obj.domain.maxCorner, ...
                    "obsMinCorners", obsMinCorners, "obsMaxCorners", obsMaxCorners, ...
                    "objectiveIntegral", sum(obj.domain.objective.values(:)));

    % Save all parameters to output file
    initsFile = strcat(obj.artifactName, "_miSimInits");
    initsFile = fullfile(matlab.project.rootProject().RootFolder, "sandbox", initsFile);
    save(initsFile, "-struct", "inits");
end
