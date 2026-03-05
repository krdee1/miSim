%% Plot AERPAW logs (trajectory, radio)
resultsPath = fullfile(matlab.project.rootProject().RootFolder, "sandbox", "t1"); % Define path to results copied from AERPAW platform

% Plot GPS logged data and scenario information (domain, objective, obstacles)
fGlobe = plotGpsLogs(resultsPath);

% Plot radio statistics
fRadio = plotRadioLogs(resultsPath);

%% Run simulation
% Run miSim using same AERPAW scenario definition CSV


%% Plot AERPAW trajectory logs onto simulated result for comparison