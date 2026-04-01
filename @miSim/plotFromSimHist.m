function obj = plotFromSimHist(obj, initsPath, histPath)
% PLOTFROMSIMHIST  Reconstruct all three miSim plots from saved matfiles.
%
% Loads the simInits matfile to rebuild domain/obstacle/objective/agent
% geometry, then loads the simHist matfile to restore the full time-history
% arrays.  Produces the same three figures that a live run would generate:
%   1. Sensor performance vs. time  (obj.fPerf)
%   2. Barrier function values vs. time  (obj.hf)
%   3. 3-D spatial figure with domain, obstacles, objective, agent trails,
%      and final-timestep communications topology  (obj.f)
%
% Usage:
%   sim = miSim;
%   sim = sim.plotFromSimHist( ...
%       'sandbox/2025_01_01_12_00_00_miSimHist.mat', ...
%       'sandbox/2025_01_01_12_00_00_miSimInits.mat');

arguments (Input)
    obj       (1, 1) {mustBeA(obj, 'miSim')};
    initsPath (1, 1) string;
    histPath  (1, 1) string;
end
arguments (Output)
    obj (1, 1) {mustBeA(obj, 'miSim')};
end

% ---- Reconstruct geometry from inits (plots disabled) --------------------
obj = obj.initializeFromInits(initsPath);
nAgents = size(obj.agents, 1);

% ---- Load history data ---------------------------------------------------
data = load(histPath);
out  = data.out;

nHistTimesteps = size(out.barriers, 2);
nPosTimesteps  = size(out.agent(1).pos, 1);

% ---- Populate barrier history --------------------------------------------
% out.barriers may be narrower than the pre-allocated obj.barriers if the
% run was shorter than maxIter; fill what we have and leave the rest NaN.
obj.barriers(:, 1:nHistTimesteps) = out.barriers;

% ---- Populate position history and advance agents to final positions -----
for ii = 1:nAgents
    agentPos = out.agent(ii).pos;           % (nPosTimesteps × 3)
    nPts = size(agentPos, 1);
    obj.posHist(ii, 1:nPts, :) = reshape(agentPos, [1, nPts, 3]);
    obj.agents{ii}.pos = agentPos(end, :);  % show final position in spatial plot
end

% ---- Set final constraint topology ---------------------------------------
obj.constraintAdjacencyMatrix = out.constraintAdjacency(:, :, end);

% ---- Recompute partitioning at final agent positions ---------------------
obj.partitioning = obj.agents{1}.partition(obj.agents, obj.domain.objective);

% ---- Enable plotting and produce spatial + barrier figures ---------------
obj.makePlots = true;
obj = obj.plot();

% ---- Performance figure (built directly — live machinery is incremental) -
nPerfTimesteps = numel(out.perf);
times = (0:nPerfTimesteps - 1) * obj.timestep;
normFactor = 1 / max(out.perf);

obj.fPerf = figure;
ax = axes(obj.fPerf);
hold(ax, "on");
title(ax,  "Sensor Performance");
xlabel(ax, "Time (s)");
ylabel(ax, "Sensor Performance");
grid(ax, "on");

legendStrings = strings(nAgents + 1, 1);
legendStrings(1) = "Total";
plot(ax, times, out.perf * normFactor, "LineWidth", 1.5);
for ii = 1:nAgents
    agentPerf  = out.agent(ii).perf;
    agentTimes = times(1:numel(agentPerf));
    plot(ax, agentTimes, agentPerf * normFactor);
    if isfield(out.agent(ii), 'label')
        legendStrings(ii + 1) = string(out.agent(ii).label);
    else
        legendStrings(ii + 1) = sprintf("Agent %d", ii);
    end
end
legend(ax, legendStrings, "Location", "northwest");
hold(ax, "off");

% Bring spatial figure to the front
figure(obj.f);

end
