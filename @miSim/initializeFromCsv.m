function obj = initializeFromCsv(obj, csvPath)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        csvPath (1, 1) string;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end

    params = obj.readScenarioCsv(tc.csvPath);

end