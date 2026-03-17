clear;
% Load data
dataPath = fullfile('.', 'sandbox', 'plot3');
simHists = dir(dataPath); simHists = simHists(3:end);
simInits = simHists(endsWith({simHists.name}, 'miSimInits.mat'));
simHists = simHists(endsWith({simHists.name}, 'miSimHist.mat'));
assert(length(simHists) == length(simInits), "input data availability mismatch");
assert(isscalar(simHists));

init = fullfile(simInits(1).folder, simInits(1).name);
hist = fullfile(simHists(1).folder, simHists(1).name);

init = load(init);
hist = load(hist);
hist = hist.out;

f3 = figure;
x3 = axes;
assert(size(init.objectivePos, 1) == 1)
assert(hist.useDoubleIntegrator);

plot(hist.perf./init.objectiveIntegral);
hold("on");
for ii = 1:length(hist.agent)
    plot(hist.agent(ii).perf./init.objectiveIntegral);
end
grid("on");
ylabel("Performance (normalized)");
xlabel("Timestep");
legend(["Cumulative"; "Agent 1"; "Agent 2"; "Agent 3"; "Agent 4"], "Location", "northwest");
title("$AII\beta$ Performance", "Interpreter", "latex");

f4 = figure;
x4 = axes;

% Compute pairwise distances between agents over time
nAgents = length(hist.agent);
commsRadius = hist.agent(1).commsRadius;
collisionRadius = hist.agent(1).collisionRadius;
nPairs = nchoosek(nAgents, 2);
T = size(hist.agent(1).pos, 1);
pairDistMat = NaN(T, nPairs);
pp = 0;
for jj = 1:nAgents-1
    for kk = jj+1:nAgents
        pp = pp + 1;
        pairDistMat(:, pp) = vecnorm(hist.agent(jj).pos - hist.agent(kk).pos, 2, 2);
    end
end

% Cap at communications range
% pairDistMat = min(pairDistMat, commsRadius);

% Plot all pairwise distances over time
hold(x4, 'on');
hLeft = gobjects(nPairs, 1);
for pp = 1:nPairs
    hLeft(pp) = plot(x4, pairDistMat(:, pp), 'LineWidth', 1);
end
yline(x4, collisionRadius, 'r--', "Label", "Collision Radius", "LabelHorizontalAlignment", "left", "HandleVisibility", "off");
yline(x4, commsRadius, 'r--', "Label", "Communications Radius", "LabelHorizontalAlignment", "left", "HandleVisibility", "off");
hold(x4, 'off');
xlabel(x4, "Timestep");
ylabel(x4, "Pairwise distance");
title(x4, "$AII\beta$ Pairwise Agent Distances and Barrier Function Values", "Interpreter", "latex");
grid(x4, "on");
ylim(x4, [0, commsRadius + 5]);

% Build legend labels
pairLabels = strings(nPairs, 1);
pp = 0;
for jj = 1:nAgents-1
    for kk = jj+1:nAgents
        pp = pp + 1;
        pairLabels(pp) = sprintf("Agents %d-%d Distance", jj, kk);
    end
end
% Left legend created after right-axis plots (see below)

% Plot all barrier function values on right Y-axis
nObs = init.numObstacles;
nAA = nchoosek(nAgents, 2);
nAO = nAgents * nObs;
nAD = nAgents * 6;
nComms = size(hist.barriers, 1) - nAA - nAO - nAD;

yyaxis(x4, 'right');
hold(x4, 'on');

% Color palettes: pairs share colors across collision/comms,
% agents share colors across obstacle/domain
pairColors = lines(nAA);
agentColors = lines(nAgents);

% Row offsets in hist.barriers
colStart = 1;
obsStart = colStart + nAA;
domStart = obsStart + nAO;
comStart = domStart + nAD;

% Collision + Comms barriers grouped per pair (same color)
hRight = gobjects(0, 1);
rightLabels = strings(0, 1);
for pp = 1:nAA
    hRight(end+1) = plot(x4, hist.barriers(colStart + pp - 1, :), '--', 'LineWidth', 1, 'Color', pairColors(pp, :));
    rightLabels(end+1) = sprintf('h_{col} %d', pp);
end
for pp = 1:nComms
    hRight(end+1) = plot(x4, hist.barriers(comStart + pp - 1, :), '-', 'LineWidth', 1.5, 'Color', pairColors(pp, :));
    rightLabels(end+1) = sprintf('h_{com} %d', pp);
end

% Obstacle barriers — colored by agent
idx = obsStart;
for aa = 1:nAgents
    for oo = 1:nObs
        hRight(end+1) = plot(x4, hist.barriers(idx, :), ':', 'LineWidth', 1, 'Color', agentColors(aa, :));
        rightLabels(end+1) = sprintf('h_{obs} a%d-o%d', aa, oo);
        idx = idx + 1;
    end
end


hold(x4, 'off');
ylabel(x4, "Barrier function $h$", "Interpreter", "latex");

% Clamp both Y-axes to start at 0
yyaxis(x4, 'left');  ylim(x4, [0, 25]);
yyaxis(x4, 'right'); ylim(x4, [0, inf]);
x4.YAxis(2).Color = 'k';

% Combined legend
legend([hLeft(:); hRight(:)], [pairLabels(:); rightLabels(:)], "Location", "eastoutside");