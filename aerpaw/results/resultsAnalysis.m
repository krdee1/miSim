%% Plot AERPAW logs (trajectory, radio)
resultsPath = fullfile(matlab.project.rootProject().RootFolder, "sandbox", "two_around_wall"); % Define path to results copied from AERPAW platform

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

% Plot GPS logged data and scenario information (domain, objective, obstacles)
seaToGroundLevel = 110; % measured approximately from USGS national map viewer
plotWholeFlight = true; % do not attempt to automatically trim initial and final positioning and landing from flight plot (buggy)
[fGlobe, G] = plotGpsLogs(resultsPath, seaToGroundLevel, true);

% Plot radio statistics
[fRadio, R] = plotRadioLogs(resultsPath);
set(findobj(fRadio, 'Type', 'axes'), 'XLim', controller.timestamp([1, end]));

%% Run simulation
% Run miSim using same AERPAW scenario definition CSV
csvPath = fullfile(matlab.project.rootProject().RootFolder, "aerpaw", "config", "scenario.csv");
params = readScenarioCsv(csvPath);

% Visualization settings
plotCommsGeometry = false;
makePlots = true;
makeVideo = true;

% Define scenario according to CSV specification
domain = rectangularPrism;
domain = domain.initialize([params.domainMin; params.domainMax], REGION_TYPE.DOMAIN, "Domain");
domain.objective = domain.objective.initialize(objectiveFunctionWrapper(params.objectivePos, reshape(params.objectiveVar, [1, 2 2])), domain, params.discretizationStep, params.protectedRange, params.sensorPerformanceMinimum);

agents = cell(size(params.initialPositions, 2) / 3, 1);
for ii = 1:size(agents, 1)
    agents{ii} = agent;

    sensorModel = sigmoidSensor;
    sensorModel = sensorModel.initialize(params.alphaDist(ii), params.betaDist(ii), params.alphaTilt(ii), params.betaTilt(ii));

    collisionGeometry = spherical;
    collisionGeometry = collisionGeometry.initialize(params.initialPositions((((ii - 1) * 3) + 1):(ii * 3)), params.collisionRadius(ii), REGION_TYPE.COLLISION, sprintf("Agent %d collision geometry", ii));

    agents{ii} = agents{ii}.initialize(params.initialPositions((((ii - 1) * 3) + 1):(ii * 3)), collisionGeometry, sensorModel, params.comRange(ii), params.maxIter, params.initialStepSize, sprintf("Agent %d", ii), plotCommsGeometry);
end

% Create obstacles
obstacles = cell(params.numObstacles, 1);
for ii = 1:size(obstacles, 1)
    obstacles{ii} = rectangularPrism;
    obstacles{ii} = obstacles{ii}.initialize([params.obstacleMin((((ii - 1) * 3) + 1):(ii * 3)); params.obstacleMax((((ii - 1) * 3) + 1):(ii * 3))], "OBSTACLE", sprintf("Obstacle %d", ii));
end

% Set up simulation
sim = miSim;
sim = sim.initialize(domain, agents, params.barrierGain, params.barrierExponent, params.minAlt, params.timestep, params.maxIter, obstacles, makePlots, makeVideo);

% Save simulation parameters to output file
sim.writeInits();

% Run
sim = sim.run();

%% Plot AERPAW trajectory logs onto simulated result for comparison
% Duplicate plot to overlay with logged trajectories
comparison = figure;
copyobj(sim.f.Children, comparison);

% Plot trajectories on top
for ii = 1:size(G, 1)
    for jj = 1:size(sim.spatialPlotIndices, 2)
        hold(comparison.Children.Children(sim.spatialPlotIndices(jj)), "on");
        plot3(comparison.Children(1).Children(sim.spatialPlotIndices(jj)), G{ii}.East, G{ii}.North, G{ii}.Up + seaToGroundLevel, 'Color', 'r', 'LineWidth', 1);
        hold(comparison.Children.Children(sim.spatialPlotIndices(jj)), "off");
    end
end