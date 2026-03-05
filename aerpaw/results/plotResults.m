% Define path to run results copied from AERPAW platform
resultsPath = fullfile(matlab.project.rootProject().RootFolder, "sandbox", "t1");

% Plot GPS logged data and scenario information (domain, objective, obstacles)
uif = plotGpsLogs(resultsPath);

% Plot radio statistics
f = plotRadioLogs(resultsPath);