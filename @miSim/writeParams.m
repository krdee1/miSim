function writeParams(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
    end

    % Collect agent parameters
    collisionRadii = cellfun(@(x) x.collisionGeometry.radius, obj.agents);
    alphaDist = cellfun(@(x) x.sensorModel.alphaDist, obj.agents);
    betaDist = cellfun(@(x) x.sensorModel.betaDist, obj.agents);
    alphaTilt = cellfun(@(x) x.sensorModel.alphaTilt, obj.agents);
    betaTilt = cellfun(@(x) x.sensorModel.alphaDist, obj.agents);
    comRange = cellfun(@(x) x.commsGeometry.radius, obj.agents);

    % Combine with simulation parameters
    params = struct('timestep', obj.timestep, 'maxIter', obj.maxIter, 'minAlt', obj.obstacles{end}.maxCorner(3), 'discretizationStep', obj.domain.objective.discretizationStep, ...
                    'collisionRadius', collisionRadii, 'alphaDist', alphaDist, 'betaDist', betaDist, ...
                    'alphaTilt', alphaTilt, 'betaTilt', betaTilt, 'comRange', comRange);

    % Save all parameters to output file
    paramsFile = strcat(obj.artifactName, "_miSimParams");
    paramsFile = fullfile(matlab.project.rootProject().RootFolder, 'sandbox', paramsFile);
    save(paramsFile, "-struct", "params");
end