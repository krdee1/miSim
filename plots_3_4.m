clear;

%% Load data
dataPath = fullfile('.', 'sandbox', 'plot3');
dataFiles = dir(dataPath);
dataFiles = dataFiles(~startsWith({dataFiles.name}, '.'));
simInits = dataFiles(endsWith({dataFiles.name}, 'miSimInits.mat'));
simHists = dataFiles(endsWith({dataFiles.name}, 'miSimHist.mat'));
assert(length(simHists) == length(simInits), "input data availability mismatch");
assert(isscalar(simHists));

init = load(fullfile(simInits(1).folder, simInits(1).name));
hist = load(fullfile(simHists(1).folder, simHists(1).name));
hist = hist.out;

%% Plot 3: Per-agent and cumulative normalized performance
assert(size(init.objectivePos, 1) == 1);
assert(hist.useDoubleIntegrator);

nAgents = length(hist.agent);
agentLabels = "Agent " + string(1:nAgents)';

f3 = figure;
x3 = axes;
hold(x3, 'on');
plot(x3, hist.perf ./ init.objectiveIntegral, "LineWidth", 2);
for ii = 1:nAgents
    plot(x3, hist.agent(ii).perf ./ init.objectiveIntegral, "LineWidth", 2);
end
hold(x3, 'off');
grid(x3, "on");
ylabel(x3, "Performance (normalized)");
xlabel(x3, "Timestep");
legend(x3, ["Cumulative"; agentLabels], "Location", "northwest");
title(x3, "$AII\beta$ Performance", "Interpreter", "latex");

savefig(f3, "plot3.fig");
exportgraphics(f3, "plot3.png");

%% Plot 4: Pairwise distances and barrier functions
commsRadius = hist.agent(1).commsRadius;
collisionRadius = hist.agent(1).collisionRadius;
nPairs = nchoosek(nAgents, 2);
T = size(hist.agent(1).pos, 1);

% Compute pairwise distances over time
pairDistMat = NaN(T, nPairs);
pairLabels = strings(nPairs, 1);
pp = 0;
for jj = 1:nAgents-1
    for kk = jj+1:nAgents
        pp = pp + 1;
        pairDistMat(:, pp) = vecnorm(hist.agent(jj).pos - hist.agent(kk).pos, 2, 2);
        pairLabels(pp) = sprintf("Agents %d-%d Distance", jj, kk);
    end
end

f4 = figure;
x4 = axes;

% Left Y-axis: pairwise distances
hold(x4, 'on');
hLeft = gobjects(nPairs, 1);
for pp = 1:nPairs
    hLeft(pp) = plot(x4, pairDistMat(:, pp), 'LineWidth', 2);
end
yline(x4, collisionRadius, 'r--', "Label", "Collision Radius", ...
    "LabelHorizontalAlignment", "left", "HandleVisibility", "off");
yline(x4, commsRadius, 'r--', "Label", "Communications Radius", ...
    "LabelHorizontalAlignment", "left", "HandleVisibility", "off");
hold(x4, 'off');
xlabel(x4, "Timestep");
ylabel(x4, "Pairwise distance");
title(x4, "$AII\beta$ Pairwise Agent Distances and Barrier Function Values", "Interpreter", "latex");
grid(x4, "on");

savefig(f4, "plot4_distanceOnly.fig");
exportgraphics(f4, "plot4_distanceOnly.png");

% Right Y-axis: barrier function values
nObs = init.numObstacles;
nAA = nchoosek(nAgents, 2);
nAO = nAgents * nObs;
nAD = nAgents * 6;
nComms = size(hist.barriers, 1) - nAA - nAO - nAD;

colStart = 1;
comStart = colStart + nAA + nAO + nAD;

pairColors = lines(nAA);

yyaxis(x4, 'right');
hold(x4, 'on');
hRight = gobjects(nAA + nComms, 1);
rightLabels = strings(nAA + nComms, 1);
idx = 0;
for pp = 1:nAA
    idx = idx + 1;
    hRight(idx) = plot(x4, hist.barriers(colStart + pp - 1, :), '--', ...
        'LineWidth', 1.5, 'Color', pairColors(pp, :));
    rightLabels(idx) = sprintf('h_{col} %d', pp);
end
for pp = 1:nComms
    idx = idx + 1;
    hRight(idx) = plot(x4, hist.barriers(comStart + pp - 1, :), '-.', ...
        'LineWidth', 1.5, 'Color', pairColors(pp, :));
    rightLabels(idx) = sprintf('h_{com} %d', pp);
end
hold(x4, 'off');
ylabel(x4, "Barrier function $h$", "Interpreter", "latex");

% Y-axis limits
yyaxis(x4, 'left');  ylim(x4, [0, 25]);
yyaxis(x4, 'right'); ylim(x4, [0, 275]);
x4.YAxis(2).Color = 'k';

legend([hLeft(:); hRight(:)], [pairLabels(:); rightLabels(:)], "Location", "eastoutside");

savefig(f4, "plot4.fig");
exportgraphics(f4, "plot4.png");
