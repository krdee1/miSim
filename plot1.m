clear;
% Load data
dataPath = fullfile('.', 'sandbox', 'plot1');
simHists = dir(dataPath); simHists = simHists(3:end);
simInits = simHists(endsWith({simHists.name}, 'miSimInits.mat'));
simHists = simHists(endsWith({simHists.name}, 'miSimHist.mat'));
assert(length(simHists) == length(simInits), "input data availability mismatch");

% Initialize plotting data
nRuns = length(simHists);
Cfinal = NaN(nRuns, 1);
n = NaN(nRuns, 1);
doubleIntegrator = NaN(nRuns, 1);
numObjective = NaN(nRuns, 1);
positions = cell(6, nRuns);
commsRadius = NaN(nRuns, 1);
collisionRadius = NaN(nRuns, 1);

% Aggregate relevant data
for ii = 1:length(simHists)
    initName = strrep(simInits(ii).name, "_miSimInits.mat", "");
    histName = strrep(simHists(ii).name, "_miSimHist.mat", "");
    assert(initName == histName);

    init = load(fullfile(simInits(ii).folder, simInits(ii).name));
    hist = load(fullfile(simHists(ii).folder, simHists(ii).name));

    % Stash relevant data
    Cfinal(ii) = hist.out.perf(end) / init.objectiveIntegral;
    n(ii) = init.numAgents;
    doubleIntegrator(ii) = init.useDoubleIntegrator;
    numObjective(ii) = size(init.objectivePos, 1);
    commsRadius(ii) = unique(init.comRange);
    collisionRadius(ii) = unique(init.collisionRadius);
    for jj = 1:length(hist.out.agent)
        alphaDist(jj, ii) = hist.out.agent(jj).sensor.alphaDist;
        positions{jj, ii} = hist.out.agent(jj).pos;
        assert(hist.out.agent(jj).commsRadius == commsRadius(ii));
        assert(hist.out.agent(jj).collisionRadius == collisionRadius(ii));
    end
end

commsRadius = unique(commsRadius); assert(isscalar(commsRadius));
collisionRadius = unique(collisionRadius); assert(isscalar(collisionRadius));
sensors = unique(alphaDist(1, :));

config = [];
for ii = 1:length(simHists)
    % number of agents
    s = num2str(n(ii));

    % number of objectives
    if numObjective(ii) == 1
        s = strcat(s, "_A");
    elseif numObjective(ii) == 2
        s = strcat(s, "_B");
    end

    % sensor pararmeter set
    if alphaDist(1, ii) == sensors(1)
        s = strcat(s, "_I");
    elseif alphaDist(1, ii) == sensors(2)
        s = strcat(s, "_II");
    end

    % agent dynamics
    if ~doubleIntegrator(ii)
        s = strcat(s, '_alpha');
    elseif doubleIntegrator(ii)
        s = strcat(s, '_beta');
    end
    config = [config; s];
end

close all;
f1 = figure;
x1 = axes;

n_unique = sort(unique(n));
C = [];
for ii = 1:length(n_unique)
    nIdx = n == n_unique(ii);
    C = [C; [Cfinal(nIdx)]']; 
end
bar(C);
xlabel("Number of agents");
ylabel("Final coverage (fraction of maximum)");
title("Final performance of parameterizations");
legend(["$AI\alpha$"; "$AI\beta$"; "$AII\alpha$"; "$BI\beta$"], "Interpreter", "latex");
grid("on");

f2 = figure;
x2 = axes;

% Compute pairwise distances between agents in each column of positions
% cell array
% Compute pairwise distances between agents
maxPairs = nchoosek(6, 2); % 15 pairs for max 6 agents
pairDist = cell(maxPairs, nRuns);
for ii = 1:nRuns
    pp = 0;
    for jj = 1:n(ii)-1
        for kk = jj+1:n(ii)
            pp = pp + 1;
            pairDist{pp, ii} = vecnorm(positions{jj, ii} - positions{kk, ii}, 2, 2);
        end
    end
end

% Compute mean, min, max pairwise distance across all pairs and timesteps per run
meanPairDist = NaN(nRuns, 1);
minPairDist = NaN(nRuns, 1);
maxPairDist = NaN(nRuns, 1);
for ii = 1:nRuns
    nPairs = nchoosek(n(ii), 2);
    D = vertcat(pairDist{1:nPairs, ii});
    meanPairDist(ii) = mean(D);
    minPairDist(ii) = min(D);
    maxPairDist(ii) = max(D);
end

% Group pairwise distance stats by n-value (same layout as bar plot)
nConfigs = nRuns / length(n_unique);
meanD = NaN(length(n_unique), nConfigs);
minD = NaN(length(n_unique), nConfigs);
maxD = NaN(length(n_unique), nConfigs);
for ii = 1:length(n_unique)
    idx = find(n == n_unique(ii));
    meanD(ii, :) = meanPairDist(idx)';
    minD(ii, :) = minPairDist(idx)';
    maxD(ii, :) = maxPairDist(idx)';
end

% Plot whiskers (min to max) with mean markers
nGroups = length(n_unique);
barWidth = 0.8;
groupWidth = barWidth / nConfigs;
hold(x2, 'on');
for jj = 1:nConfigs
    xPos = (1:nGroups) + (jj - (nConfigs + 1) / 2) * groupWidth;
    errorbar(x2, xPos, meanD(:, jj), meanD(:, jj) - minD(:, jj), maxD(:, jj) - meanD(:, jj), ...
        'o', 'LineWidth', 1.5, 'MarkerSize', 6, 'CapSize', 10);
end
hold(x2, 'off');
set(x2, 'XTick', 1:nGroups, 'XTickLabel', string(n_unique));
xlabel(x2, "Number of agents");
ylabel(x2, "Pairwise distance");
title(x2, "Pairwise Agent Distances (min/mean/max)");
legend(x2, ["$AI\alpha$"; "$AI\beta$"; "$AII\alpha$"; "$BI\beta$"], "Interpreter", "latex");
grid(x2, "on");

yline(collisionRadius, 'r--', "Label", "Collision Radius", "LabelHorizontalAlignment", "left", "HandleVisibility", "off");
yline(commsRadius, 'r--', "Label", "Communications Radius", "LabelHorizontalAlignment", "left", "HandleVisibility", "off");

ylim([0, commsRadius + 5]);