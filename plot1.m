clear;
% Load data
dataPath = fullfile('.', 'sandbox', 'plot1');
simHists = dir(dataPath); simHists = simHists(3:end);
simInits = simHists(endsWith({simHists.name}, 'miSimInits.mat'));
simHists = simHists(endsWith({simHists.name}, 'miSimHist.mat'));
assert(length(simHists) == length(simInits), "input data availability mismatch");

% Initialize plotting data
Cfinal = NaN(12, 1);
n = NaN(12, 1);
doubleIntegrator = NaN(12, 1);
numObjective = NaN(12, 1);

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
    for jj = 1:length(hist.out.agent)
        alphaDist(jj, ii) = hist.out.agent(jj).sensor.alphaDist;
    end
end

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
f = figure;
x = axes; grid(x, "on");

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
keyboard

