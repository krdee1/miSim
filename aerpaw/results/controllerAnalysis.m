function controller = controllerAnalysis(resultsPath)
    arguments (Input)
        resultsPath (1, 1) string;
    end
    arguments (Output)
        controller table;
    end
    
    % Measure intervals between issuing commands from the controller 
    % (make sure this is ~4-5 seconds at minimum to avoid overwhelming the UAV autopilot)
    r = dir(resultsPath);
    controllerPath = fullfile(r(startsWith({r.name}, 'controller_')).folder, r(startsWith({r.name}, 'controller_')).name);
    controllerPath = dir(controllerPath);
    controllerPath = fullfile(controllerPath(endsWith({controllerPath.name}, '_controller_log.txt')).folder, controllerPath(endsWith({controllerPath.name}, '_controller_log.txt')).name);
    controller = readControllerLogs(controllerPath);
    rpIdx = startsWith(controller.message, "Iteration duration: ");
    s = split(controller.message(rpIdx), "Iteration duration: ");
    s = split(s(:, 2), ' s');
    s = duration(strcat("00:", s(:, 1)), "InputFormat", "mm:ss.SSS");
    s.Format = "mm:ss.SSS";
    fprintf("Minimum command spacing: %2.3f seconds\n", seconds(min(s)));
    fprintf("Maximum command spacing: %2.3f seconds\n", seconds(max(s)));
    fprintf("Mean command spacing: %2.3f seconds\n", seconds(mean(s)));
    fprintf("Median command spacing: %2.3f seconds\n", seconds(median(s)));
    if seconds(min(s)) < 4 
        warning("Minimum command spacing %2.3f questionably short for AERPAW", seconds(min(s)));
    end
end