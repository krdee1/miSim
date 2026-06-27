%% Plot AERPAW logs (trajectory, radio)
resultsPath = fullfile(matlab.project.rootProject().RootFolder, "sandbox", "columns_simulated"); % Define path to results copied from AERPAW platform

% Check timeline in controller logs
controller = controllerAnalysis(resultsPath);

% Plot GPS logged data and scenario information (domain, objective, obstacles)
seaToGroundLevel = 110; % measured approximately from USGS national map viewer
plotWholeFlight = true; % do not attempt to automatically trim initial and final positioning and landing from flight plot (buggy)
[fGlobe, G] = plotGpsLogs(resultsPath, seaToGroundLevel, true);

% Plot radio statistics (time-based and distance-based)
[fRadio, fRadioDist, R] = plotRadioLogs(resultsPath, G, controller.timestamp([1, end]));

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
if length(params.objectiveVar) > 4 && length(params.objectivePos) > 2
    objectiveSigma = permute(reshape(params.objectiveVar, [length(params.objectiveVar)/4 2 2]), [3 1 2]);
    objectivePos = reshape(params.objectivePos, [length(params.objectivePos)/2, 2])';
else
    objectiveSigma = reshape(params.objectiveVar, [1, 2, 2]);
    objectivePos = params.objectivePos;
end
domain.objective = domain.objective.initialize(objectiveFunctionWrapper(objectivePos, objectiveSigma), domain, params.discretizationStep, params.protectedRange, params.sensorPerformanceMinimum);

agents = cell(size(params.initialPositions, 2) / 3, 1);
for ii = 1:size(agents, 1)
    agents{ii} = agent;

    sensorModel = sigmoidSensor;
    sensorModel = sensorModel.initialize(params.alphaDist(ii), params.betaDist(ii), params.alphaTilt(ii), params.betaTilt(ii));

    collisionGeometry = spherical;
    collisionGeometry = collisionGeometry.initialize(params.initialPositions((((ii - 1) * 3) + 1):(ii * 3)), params.collisionRadius(ii), REGION_TYPE.COLLISION, sprintf("Agent %d collision geometry", ii));

    agents{ii} = agents{ii}.initialize(params.initialPositions((((ii - 1) * 3) + 1):(ii * 3)), collisionGeometry, sensorModel, params.comRange(ii), params.maxIter, params.initialStepSize, 5.0, sprintf("Agent %d", ii), plotCommsGeometry);
    if isfield(params, 'txPower')
        agents{ii}.txPower = params.txPower(ii); % SINR comms model transmit power (W)
    end
end

% Create obstacles
obstacles = cell(params.numObstacles, 1);
for ii = 1:size(obstacles, 1)
    obstacles{ii} = rectangularPrism;
    obstacles{ii} = obstacles{ii}.initialize([params.obstacleMin((((ii - 1) * 3) + 1):(ii * 3)); params.obstacleMax((((ii - 1) * 3) + 1):(ii * 3))], "OBSTACLE", sprintf("Obstacle %d", ii));
end

% SINR communications model parameters (optional CSV columns; default to the
% fixed-radius path so existing scenarios are unaffected)
if isfield(params, 'useSinrComms');     useSinrComms = logical(params.useSinrComms); else; useSinrComms = false; end
if isfield(params, 'sinrThreshold');    sinrThreshold = params.sinrThreshold;        else; sinrThreshold = 0;        end
if isfield(params, 'pathLossExponent'); pathLossExponent = params.pathLossExponent;  else; pathLossExponent = 2.0;    end
if isfield(params, 'ambientTemp');      ambientTemp = params.ambientTemp;            else; ambientTemp = 290.0;      end
if isfield(params, 'centerFreq');       centerFreq = params.centerFreq;              else; centerFreq = 2.4e9;       end
if isfield(params, 'bandwidth');        bandwidth = params.bandwidth;                else; bandwidth = 20e6;         end

% Set up simulation. The four args after makeVideo (useDoubleIntegrator,
% dampingCoeff, useFixedTopology, optimizeSensorPointing) are kept at their
% prior effective defaults to preserve existing analysis behavior; only the
% SINR comms parameters are threaded through from the scenario.
sim = miSim;
sim = sim.initialize(domain, agents, params.barrierGain, params.barrierExponent, params.minAlt, params.timestep, params.maxIter, obstacles, makePlots, makeVideo, ...
                     false, 2.0, false, false, useSinrComms, sinrThreshold, pathLossExponent, ambientTemp, centerFreq, bandwidth);

% Save simulation parameters to output file
sim.writeInits();

% Run
sim = sim.run();

% Save results
sim = sim.teardown();

%% Plot AERPAW trajectory logs onto simulated result for comparison
% Duplicate plot to overlay with logged trajectories
comparison = figure;
copyobj(sim.f.Children, comparison);

% Plot trajectories on top
for ii = 1:size(G, 1)
    gpsTimes = G{ii}.Timestamp;
    gpsTimes.TimeZone = '';
    inRange = gpsTimes >= controller.timestamp(1) & gpsTimes <= controller.timestamp(end);
    for jj = 1:size(sim.spatialPlotIndices, 2)
        hold(comparison.Children.Children(sim.spatialPlotIndices(jj)), "on");
        plot3(comparison.Children(1).Children(sim.spatialPlotIndices(jj)), G{ii}.East(inRange), G{ii}.North(inRange), G{ii}.Up(inRange) + seaToGroundLevel, 'Color', 'r', 'LineWidth', 1);
        hold(comparison.Children.Children(sim.spatialPlotIndices(jj)), "off");
    end
end