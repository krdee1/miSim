clear;

%% Load data
dataPath = fullfile('.', 'sandbox', 'plot1');
dataFiles = dir(dataPath);
dataFiles = dataFiles(~startsWith({dataFiles.name}, '.'));
simInits = dataFiles(endsWith({dataFiles.name}, 'miSimInits.mat'));
simHists = dataFiles(endsWith({dataFiles.name}, 'miSimHist.mat'));
assert(length(simHists) == length(simInits), "input data availability mismatch");

%% Aggregate run data
nRuns = length(simHists);
Cfinal = NaN(nRuns, 1);
nAgents = NaN(nRuns, 1);
doubleIntegrator = NaN(nRuns, 1);
numObjective = NaN(nRuns, 1);
commsRadius = NaN(nRuns, 1);
collisionRadius = NaN(nRuns, 1);
maxAgents = 6;
alphaDist = NaN(maxAgents, nRuns);
positions = cell(maxAgents, nRuns);
adjacency = cell(nRuns, 1);

for ii = 1:nRuns
    initName = strrep(simInits(ii).name, "_miSimInits.mat", "");
    histName = strrep(simHists(ii).name, "_miSimHist.mat", "");
    assert(initName == histName);

    init = load(fullfile(simInits(ii).folder, simInits(ii).name));
    hist = load(fullfile(simHists(ii).folder, simHists(ii).name));

    Cfinal(ii) = hist.out.perf(end) / init.objectiveIntegral;
    nAgents(ii) = init.numAgents;
    doubleIntegrator(ii) = init.useDoubleIntegrator;
    numObjective(ii) = size(init.objectivePos, 1);
    commsRadius(ii) = unique(init.comRange);
    collisionRadius(ii) = unique(init.collisionRadius);

    adjacency{ii} = hist.out.constraintAdjacency(:, :, 1);
    for jj = 1:nAgents(ii)
        alphaDist(jj, ii) = hist.out.agent(jj).sensor.alphaDist;
        positions{jj, ii} = hist.out.agent(jj).pos;
        assert(hist.out.agent(jj).commsRadius == commsRadius(ii));
        assert(hist.out.agent(jj).collisionRadius == collisionRadius(ii));
    end
end

commsRadius = unique(commsRadius); assert(isscalar(commsRadius));
collisionRadius = unique(collisionRadius); assert(isscalar(collisionRadius));
sensorTypes = flip(unique(alphaDist(1, :)));
nValues = sort(unique(nAgents));
nGroups = length(nValues);

%% Build config labels
baseConfig = strings(nRuns, 1);
for ii = 1:nRuns
    s = "";
    if numObjective(ii) == 1
        s = s + "A";
    elseif numObjective(ii) == 2
        s = s + "B";
    end
    if alphaDist(1, ii) == sensorTypes(1)
        s = s + "_I";
    elseif alphaDist(1, ii) == sensorTypes(2)
        s = s + "_II";
    end
    if ~doubleIntegrator(ii)
        s = s + "_alpha";
    else
        s = s + "_beta";
    end
    baseConfig(ii) = s;
end
configOrder = unique(baseConfig(nAgents == nValues(1)), 'stable');
nConfigs = length(configOrder);
configLabels = ["$AI\alpha$"; "$AI\beta$"; "$AII\alpha$"; "$BI\beta$"];

%% Plot 1: Final normalized coverage
close all;
f1 = figure;
x1 = axes;

C_mean = NaN(nGroups, nConfigs);
C_var = NaN(nGroups, nConfigs);
for ii = 1:nGroups
    for jj = 1:nConfigs
        mask = (nAgents == nValues(ii)) & (baseConfig == configOrder(jj));
        C_mean(ii, jj) = mean(Cfinal(mask));
        C_var(ii, jj) = var(Cfinal(mask));
    end
end

bar(x1, C_mean);
set(x1, 'XTickLabel', string(nValues));
xlabel(x1, "Number of agents");
ylabel(x1, "Final coverage (normalized)");
title(x1, "Final performance of parameterizations");
legend(x1, configLabels, "Interpreter", "latex", "Location", "northwest");
grid(x1, "on");
ylim(x1, [0, 1/2]);

savefig(f1, "plot1.fig");
exportgraphics(f1, "plot1.png");

%% Plot 2: Pairwise agent distances
f2 = figure;
x2 = axes;

% Compute pairwise distances only for connected agents (static topology)
maxPairs = nchoosek(maxAgents, 2);
pairDist = cell(maxPairs, nRuns);
for ii = 1:nRuns
    A = adjacency{ii};
    pp = 0;
    for jj = 1:nAgents(ii)-1
        for kk = jj+1:nAgents(ii)
            pp = pp + 1;
            if A(jj, kk)
                pairDist{pp, ii} = vecnorm(positions{jj, ii} - positions{kk, ii}, 2, 2);
            end
        end
    end
end

% Per-run statistics across all pairs and timesteps
meanPairDist = NaN(nRuns, 1);
minPairDist = NaN(nRuns, 1);
maxPairDist = NaN(nRuns, 1);
for ii = 1:nRuns
    nPairs = nchoosek(nAgents(ii), 2);
    D = vertcat(pairDist{1:nPairs, ii});
    meanPairDist(ii) = mean(D, "omitmissing");
    minPairDist(ii) = min(D);
    maxPairDist(ii) = max(D);
end

% Aggregate across trials per (n, config) group
meanD = NaN(nGroups, nConfigs);
minD = NaN(nGroups, nConfigs);
maxD = NaN(nGroups, nConfigs);
for ii = 1:nGroups
    for jj = 1:nConfigs
        mask = (nAgents == nValues(ii)) & (baseConfig == configOrder(jj));
        meanD(ii, jj) = mean(meanPairDist(mask));
        minD(ii, jj) = min(minPairDist(mask));
        maxD(ii, jj) = max(maxPairDist(mask));
    end
end

% Plot whiskers (min to max) with mean markers
barWidth = 0.8;
groupWidth = barWidth / nConfigs;
hold(x2, 'on');
for jj = 1:nConfigs
    xPos = (1:nGroups) + (jj - (nConfigs + 1) / 2) * groupWidth;
    errorbar(x2, xPos, meanD(:, jj), meanD(:, jj) - minD(:, jj), maxD(:, jj) - meanD(:, jj), ...
        'o', 'LineWidth', 1.5, 'MarkerSize', 6, 'CapSize', 10);
end
hold(x2, 'off');
set(x2, 'XTick', 1:nGroups, 'XTickLabel', string(nValues));
xlabel(x2, "Number of agents");
ylabel(x2, "Pairwise distance");
title(x2, "Pairwise Agent Distances (min/mean/max)");
legend(x2, configLabels, "Interpreter", "latex");
grid(x2, "on");
yline(x2, collisionRadius, 'r--', "Label", "Collision Radius", ...
    "LabelHorizontalAlignment", "left", "HandleVisibility", "off");
yline(x2, commsRadius, 'r--', "Label", "Communications Radius", ...
    "LabelHorizontalAlignment", "left", "HandleVisibility", "off");
ylim(x2, [0, commsRadius + 5]);

savefig(f2, "plot2.fig");
exportgraphics(f2, "plot2.png");
