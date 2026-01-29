function writeParams(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
    end

    % Collect agent parameters
    collisionRadii = cellfun(@(x) x.collisionGeometry.radius, obj.agents);
    alphaDist = cellfun(@(x) x.sensorModel.alphaDist, obj.agents);
    betaDist = cellfun(@(x) x.sensorModel.betaDist, obj.agents);
    alphaTilt = cellfun(@(x) x.sensorModel.alphaTilt, obj.agents);
    betaTilt = cellfun(@(x) x.sensorModel.alphaDist, obj.agents);
    comRanges = cellfun(@(x) x.commsGeometry.radius, obj.agents);
    initialStepSize = cellfun(@(x) x.initialStepSize, obj.agents);
    pos = cell2mat(cellfun(@(x) x.pos, obj.agents, 'UniformOutput', false));


    % Combine with simulation parameters
    inits = struct("timestep", obj.timestep, "maxIter", obj.maxIter, "minAlt", obj.obstacles{end}.maxCorner(3), ...
                    "discretizationStep", obj.domain.objective.discretizationStep, "protectedRange", obj.domain.objective.protectedRange, ...
                    "sensorPerformanceMinimum", obj.domain.objective.sensorPerformanceMinimum, "initialStepSize", initialStepSize, ...
                    "barrierGain", obj.barrierGain, "barrierExponent", obj.barrierExponent, "numObstacles", size(obj.obstacles, 1), ...
                    "numAgents", size(obj.agents, 1), "collisionRadius", collisionRadii, "comRange", comRanges, "alphaDist", alphaDist, ...
                    "betaDist", betaDist, "alphaTilt", alphaTilt, "betaTilt", betaTilt, ...
                    ... % ^^^ PARAMETERS ^^^ | vvv STATES vvv
                    "pos", pos);

    % Save all parameters to output file
    initsFile = strcat(obj.artifactName, "_miSimInits");
    initsFile = fullfile(matlab.project.rootProject().RootFolder, "sandbox", initsFile);
    save(initsFile, "-struct", "inits");
end