function scenario = readScenarioCsv(obj, csvPath)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        csvPath (1, 1) string;
    end
    arguments (Output)
        scenario struct;
    end

    % File input validation
    assert(isfile(csvPath), "%s is not a valid filepath.");
    assert(endsWith(csvPath, ".csv"), "%s is not a CSV file.");

    % Read file
    csv = readtable(csvPath, "TextType", "String", "NumHeaderLines", 0, "VariableNamingRule", "Preserve");
    csv.Properties.VariableNames = ["timestep", "maxIter", "minAlt", "discretizationStep", "protectedRange", "sensorPerformanceMinimum", "initialStepSize", "barrierGain", "barrierExponent", "numObstacles", "numAgents", "collisionRadius", "comRange", "alphaDist", "betaDist", "alphaTilt", "betaTilt"];
    
    for ii = 1:size(csv.Properties.VariableNames, 2)
        csv.(csv.Properties.VariableNames{ii}) = cell2mat(cellfun(@(x) str2num(x), csv.(csv.Properties.VariableNames{ii}), "UniformOutput", false));
    end
    
    % Put params into standard structure
    scenario = struct("timestep", csv.timestep, "maxIter", csv.maxIter, "minAlt", csv.minAlt, "discretizationStep", csv.discretizationStep, ...
                      "protectedRange", csv.protectedRange, "sensorPerformanceMinimum", csv.sensorPerformanceMinimum, "initialStepSize", csv.initialStepSize, ...
                      "barrierGain", csv.barrierGain, "barrierExponent", csv.barrierExponent, "numObstacles", csv.numObstacles,...
                      "numAgents", csv.numAgents, "collisionRadius", csv.collisionRadius, "comRange", csv.comRange, "alphaDist", csv.alphaDist, ...
                      "betaDist", csv.betaDist, "alphaTilt", csv.alphaTilt, "betaTilt", csv.betaTilt);

    % size check
    fns = fieldnames(scenario);
    for ii = 2:size(fns, 1)
        assert(size(scenario.(fns{ii}), 1) == size(scenario.(fns{ii - 1}), 1), "Mismatched number of rows in scenario definition CSV");
    end
end