%% Plot AERPAW logs (trajectory, radio)
resultsPath = fullfile(matlab.project.rootProject().RootFolder, "sandbox", "L1U1_simulated"); % Define path to results copied from AERPAW platform
csvPath = fullfile(matlab.project.rootProject().RootFolder, "aerpaw", "config", "scenario_service_L1U1.csv"); % scenario CSV that was flown (copied to config/scenario.csv on AERPAW)

% Check timeline in controller logs
controller = controllerAnalysis(resultsPath);

% Plot GPS logged data and scenario information (domain, objective, obstacles)
seaToGroundLevel = 110; % measured approximately from USGS national map viewer
plotWholeFlight = true; % do not attempt to automatically trim initial and final positioning and landing from flight plot (buggy)
[fGlobe, G] = plotGpsLogs(resultsPath, seaToGroundLevel, plotWholeFlight, csvPath);

% Plot radio statistics (time-based and distance-based)
[fRadio, fRadioDist, R] = plotRadioLogs(resultsPath, G, controller.timestamp([1, end]));

%% Run simulation
% Run miSim on the same AERPAW scenario definition CSV, through the same
% builder the controller's MATLAB path uses (sensing or wireless service,
% fixed-range or SINR links, single or double integrator, as the CSV says)
makePlots = true;
makeVideo = true;
sim = miSim;
sim = sim.initializeFromCsv(csvPath, makePlots, makeVideo);

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