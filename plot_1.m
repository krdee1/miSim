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

    alphaDist2 = unique(alphaDist);
    if length(alphaDist2) > 1
        alphaDist2 = alphaDist2(1);
    end
    if doubleIntegrator(ii) && all(alphaDist(1:n(ii), ii) == alphaDist2) && numObjective(ii) == 1
        a2betaIdx = ii;
        a2beta = struct("init", init, "hist", hist.out);
    end
end

commsRadius = unique(commsRadius); assert(isscalar(commsRadius));
collisionRadius = unique(collisionRadius); assert(isscalar(collisionRadius));
sensors = flip(unique(alphaDist(1, :)));
n_unique = sort(unique(n));
nGroups = length(n_unique);

% Build config label for each run
config = strings(nRuns, 1);
baseConfig = strings(nRuns, 1);
for ii = 1:nRuns
    s = "";
    if numObjective(ii) == 1
        s = s + "A";
    elseif numObjective(ii) == 2
        s = s + "B";
    end
    if alphaDist(1, ii) == sensors(1)
        s = s + "_I";
    elseif alphaDist(1, ii) == sensors(2)
        s = s + "_II";
    end
    if ~doubleIntegrator(ii)
        s = s + "_alpha";
    else
        s = s + "_beta";
    end
    baseConfig(ii) = s;
    config(ii) = n(ii) + "_" + s;
end
configOrder = unique(baseConfig(n == n_unique(1)), 'stable');
nConfigsPerN = length(configOrder);

%%
close all;
f1 = figure;
x1 = axes;

C_mean = NaN(nGroups, nConfigsPerN);
C_var = NaN(nGroups, nConfigsPerN);
for ii = 1:nGroups
    for jj = 1:nConfigsPerN
        mask = (n == n_unique(ii)) & (baseConfig == configOrder(jj));
        C_mean(ii, jj) = mean(Cfinal(mask));
        C_var(ii, jj) = var(Cfinal(mask));
    end
end

hBar = bar(x1, C_mean);
hold(x1, 'on');
for jj = 1:nConfigsPerN
    xPos = hBar(jj).XEndPoints;
    errorbar(x1, xPos, C_mean(:, jj), C_var(:, jj), 'k.', 'LineWidth', 1, 'HandleVisibility', 'off');  % disabled the error bars because they are small to the point of meaninglessness
end
hold(x1, 'off');
set(x1, 'XTickLabel', string(n_unique));
xlabel("Number of agents");
ylabel("Final coverage (normalized)");
title("Final performance of parameterizations");
legend(["$AI\alpha$"; "$AI\beta$"; "$AII\alpha$"; "$BI\beta$"], "Interpreter", "latex", "Location", "northwest");
grid("on");
ylim([0, 1/2]);

savefig(f1, "plot1.fig");
exportgraphics(f1, "plot1.png");

%%
f2 = figure;
x2 = axes;

% Compute pairwise distances between agents
maxPairs = nchoosek(6, 2);
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

% Cap pairwise distances at communications range
for ii = 1:nRuns
    nPairs = nchoosek(n(ii), 2);
    for pp = 1:nPairs
        pairDist{pp, ii} = min(pairDist{pp, ii}, commsRadius);
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

% Group pairwise distance stats by (n, config), aggregating across reps
meanD = NaN(nGroups, nConfigsPerN);
minD = NaN(nGroups, nConfigsPerN);
maxD = NaN(nGroups, nConfigsPerN);
for ii = 1:nGroups
    for jj = 1:nConfigsPerN
        mask = (n == n_unique(ii)) & (baseConfig == configOrder(jj));
        meanD(ii, jj) = mean(meanPairDist(mask));
        minD(ii, jj) = min(minPairDist(mask));
        maxD(ii, jj) = max(maxPairDist(mask));
    end
end

% Plot whiskers (min to max) with mean markers
barWidth = 0.8;
groupWidth = barWidth / nConfigsPerN;
hold(x2, 'on');
for jj = 1:nConfigsPerN
    xPos = (1:nGroups) + (jj - (nConfigsPerN + 1) / 2) * groupWidth;
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

savefig(f2, "plot2.fig");
exportgraphics(f2, "plot2.png");