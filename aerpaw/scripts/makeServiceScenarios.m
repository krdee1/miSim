function S = makeServiceScenarios(opts)
% MAKESERVICESCENARIOS  Write the 2-UAV wireless-service AERPAW scenario CSVs.
%
% Builds the three service scenarios of the ABS deployment paper (layouts
% L1-L3 paired with user densities U1-U3, Sec. VII and Table I), adapted to
% the AERPAW testbed:
%   - 2 UAVs: UAV 1 wide beam (delta 24, 0.5 mW), UAV 2 narrow beam
%     (delta 96, 2 mW)
%   - antennas fixed at nadir, single-integrator dynamics, SINR backhaul
%   - 100 m ceiling: obstacle heights are clipped to it
%   - starts drawn once (seed 11) with the paper's rejection rule at 50-70 m
%     altitude, clear of the obstacles of all three layouts, and shared by
%     all three scenarios
%   - the whole scenario shifted 35 m west of the ENU origin (geofence margin)
%
% Geometry and the start draw are defined in the paper's frame (domain from
% the origin); writeScenario translates domain, densities, obstacles and
% starts by P.shift. 35 m is a multiple of the 2.5 m grid, so the shifted
% experiment is an exact translation of the unshifted one.
%
% Every parameter lives in this file. The start draw replicates
% paperExperiments.randomStarts (same rand call order), with only the
% altitude range and fleet size changed.
%
% Usage:
%   S = makeServiceScenarios();                 % writes aerpaw/config/scenario_service_*.csv
%   S = makeServiceScenarios(Write=false);      % draw and report only
arguments
    opts.Write (1, 1) logical = true
    opts.OutDir (1, 1) string = fullfile(fileparts(fileparts(mfilename("fullpath"))), "config")
end

%% Parameters (paper Table I unless noted)
P.timestep = 1;              % s, update period
P.maxIter = 240;             % user choice: 240 steps (paper: 120 s at 1 s per step). The
                             % flown steps fall ~1 m short each (arrival tolerance), so the
                             % extra steps let the flight reach where the 120-step sim ends
P.minAlt = 30;               % m
P.gridSpacing = 2.5;         % m
P.protectedRange = 1;        % m, sensingObjective guard distance (paper code value)
P.initialStepSize = 5;       % m, decays linearly to 0
P.barrierGain = 0.4;         % kappa_s
P.barrierExponent = 1;
P.safetyRadius = 4;          % m
P.domainMin = [0, 0, 0];
P.domainMax = [250, 250, 100]; % user choice: 100 m ceiling (paper: 150 m)
P.shift = [-35, 0, 0];       % m ENU, user choice: 35 m west for geofence margin
P.partitionMin = -Inf;
P.useDoubleIntegrator = 0;   % single integrator
P.dampingCoeff = 1.5;        % unused with the single integrator
P.useFixedTopology = 0;      % lesser neighbor (same as CALSN for 2 UAVs)

% Backhaul (UAV-UAV SINR links)
P.backhaulTxPower = 0.2;     % W
P.useSinrComms = 1;
P.sinrThreshold = -16;       % dB (CBF holds +3 dB margin)
P.backhaulLossExponent = 2;
P.backhaulTemp = 290;        % K
P.backhaulFreq = 5.8e9;      % Hz
P.backhaulBandwidth = 40e6;  % Hz

% Service (rfSensor), UAV 1 wide, UAV 2 narrow (user choice)
P.serviceTxPower = [5e-4, 2e-3];   % W
P.beamwidthExponent = [24, 96];    % delta_n
P.serviceTilt = [0, 0];            % deg, nadir (user choice)
P.serviceAzimuth = [0, 0];         % deg
P.serviceRxGain = 0;               % dBi
P.serviceLossExponent = 2.8;
P.serviceTemp = 290;               % K
P.serviceFreq = 3.5e9;             % Hz
P.serviceBandwidth = 20e6;         % Hz

% Placeholders: required by the CSV parsers, ignored for rfSensor + SINR comms
P.comRange = [50, 50];
P.alphaDist = [80, 50];
P.betaDist = [0.25, 1.0];
P.alphaTilt = [8, 25];
P.betaTilt = [0.1, 0.02];

% Densities: equally weighted Gaussians, cov rows [var_x, var_y, cov_xy] (m^2)
D.U1.mu = [175, 205; 210, 160];
D.U1.cov = [1296, 1296, 0; 1600, 900, 0];
D.U2.mu = [155, 200; 205, 175];
D.U2.cov = [900, 900, 0; 1800, 1800, 600];
D.U3.mu = [150, 215; 205, 205; 195, 150];
D.U3.cov = [1024, 1024, 0; 1296, 1296, 0; 2304, 1024, 0];

% Layouts: rows [x_min, y_min, z_min, x_max, y_max, z_max] (paper heights)
L.L1 = [80, 110, 0, 100, 130, 150;
        125, 85, 0, 145, 105, 90];
L.L2 = [85, 112, 0, 105, 132, 150;
        115, 82, 0, 135, 102, 100;
        140, 50, 0, 158, 68, 150];
L.L3 = [70, 100, 0, 88, 118, 120;
        98, 70, 0, 116, 88, 150;
        105, 105, 0, 120, 120, 150;
        140, 100, 0, 152, 112, 90;
        180, 180, 0, 190, 190, 70];
for name = string(fieldnames(L))'
    L.(name)(:, 6) = min(L.(name)(:, 6), P.domainMax(3)); % clip to the ceiling
end

% Start rule (paper), altitude range changed by user choice
st.seed = 11;
st.numAgents = 2;
st.xyRange = [10, 85];
st.centralMin = 45;
st.zRange = [50, 70];        % user choice (paper: 90-110 m)
st.minSeparation = 13;
st.obstacleClearance = 10;
st.maxDraws = 20000;

scenarios = ["L1", "U1"; "L2", "U2"; "L3", "U3"];

%% Shared start draw
[pos, nDraws] = drawStarts(P, D, L, st);
fprintf("Start draw: seed %d, accepted on draw %d\n", st.seed, nDraws);
for ii = 1:size(pos, 1)
    fprintf("  UAV %d: (%.3f, %.3f, %.3f) before the shift, (%.3f, %.3f, %.3f) written\n", ii, pos(ii, :), pos(ii, :) + P.shift);
end

%% Write CSVs
S = struct("name", {}, "layout", {}, "density", {}, "file", {}, "pos", {});
for ss = 1:size(scenarios, 1)
    lay = scenarios(ss, 1);
    den = scenarios(ss, 2);
    file = fullfile(opts.OutDir, "scenario_service_" + lay + den + ".csv");
    if opts.Write
        writeScenario(file, P, D.(den), L.(lay), pos);
        fprintf("Wrote %s\n", file);
    end
    S(ss) = struct("name", lay + den, "layout", lay, "density", den, "file", file, "pos", pos);
end
end

% -------------------------------------------------------------------------
function [pos, nDraws] = drawStarts(P, D, L, st)
% Rejection-sample starts in the L-shaped lower-left region, clear of every
% layout's obstacles (replicates paperExperiments.randomStarts)
prevRng = rng(st.seed);
cleanup = onCleanup(@() rng(prevRng));
n = st.numAgents;
lo = st.xyRange(1);
hi = st.xyRange(2);
cMin = st.centralMin;
corners = [L.L1; L.L2; L.L3];

% Validation sim pieces: U1 domain, L1 obstacles (as in the paper code)
domain = buildDomain(P, D.U1);
obs = buildObstacles(L.L1);
for nDraws = 1:st.maxDraws
    pos = zeros(n, 3);
    ii = 0;
    while ii < n
        p = [lo + (hi - lo) * rand(1, 2), st.zRange(1) + diff(st.zRange) * rand];
        if p(1) >= cMin && p(2) >= cMin
            continue;
        end
        ii = ii + 1;
        pos(ii, :) = p;
    end
    Dm = sqrt(sum((permute(pos, [1, 3, 2]) - permute(pos, [3, 1, 2])).^2, 3));
    if min(Dm(~eye(n))) < st.minSeparation
        continue;
    end
    gap = inf;
    for oo = 1:size(corners, 1)
        q = min(max(pos, corners(oo, 1:3)), corners(oo, 4:6));
        gap = min(gap, min(vecnorm(pos - q, 2, 2)));
    end
    if gap < st.obstacleClearance
        continue;
    end
    % The farthest pair become UAVs 1 and n, the gateways
    [~, idx] = max(Dm(:));
    [a, b] = ind2sub([n, n], idx);
    pos = pos([a, setdiff(1:n, [a, b]), b], :);
    try
        sim = buildSim(P, domain, pos, obs);
    catch
        continue;
    end
    if all(sim.adjacency(~eye(n)))
        return;
    end
end
error("makeServiceScenarios:noValidStart", "No valid start after %d draws", st.maxDraws);
end

function domain = buildDomain(P, den)
domain = rectangularPrism;
domain = domain.initialize([P.domainMin; P.domainMax], REGION_TYPE.DOMAIN, "Domain");
[mu, sigma] = densityParams(den);
domain.objective = domain.objective.initialize(objectiveFunctionWrapper(mu, sigma), domain, ...
    P.gridSpacing, P.protectedRange, P.partitionMin, mu, sigma);
end

function [mu, sigma] = densityParams(den)
mu = den.mu;
c = den.cov;
sigma = zeros(size(mu, 1), 2, 2);
sigma(:, 1, 1) = c(:, 1);
sigma(:, 2, 2) = c(:, 2);
sigma(:, 1, 2) = c(:, 3);
sigma(:, 2, 1) = c(:, 3);
end

function obs = buildObstacles(corners)
obs = cell(size(corners, 1), 1);
for ii = 1:size(corners, 1)
    obs{ii} = rectangularPrism;
    obs{ii} = obs{ii}.initialize([corners(ii, 1:3); corners(ii, 4:6)], REGION_TYPE.OBSTACLE, sprintf("Obstacle %d", ii));
end
end

function sim = buildSim(P, domain, pos, obs)
% Same construction as initializeFromCsv for a service scenario
n = size(pos, 1);
agents = cell(n, 1);
for ii = 1:n
    sensor = rfSensor;
    sensor = sensor.initialize(P.serviceTxPower(ii), P.serviceTemp, P.serviceBandwidth, P.serviceFreq, ...
        P.serviceRxGain, P.beamwidthExponent(ii), P.serviceTilt(ii), P.serviceAzimuth(ii), P.serviceLossExponent);
    geom = spherical;
    geom = geom.initialize(pos(ii, :), P.safetyRadius, REGION_TYPE.COLLISION, sprintf("UAV %d Collision", ii));
    agents{ii} = agent;
    agents{ii} = agents{ii}.initialize(pos(ii, :), geom, sensor, P.comRange(ii), P.maxIter, ...
        P.initialStepSize, 5.0, sprintf("UAV %d", ii), false, P.backhaulTxPower);
end
sim = miSim;
sim = sim.initialize(domain, agents, P.barrierGain, P.barrierExponent, P.minAlt, P.timestep, P.maxIter, obs, ...
    false, false, logical(P.useDoubleIntegrator), P.dampingCoeff, logical(P.useFixedTopology), false, ...
    logical(P.useSinrComms), P.sinrThreshold, P.backhaulLossExponent, P.backhaulTemp, P.backhaulFreq, P.backhaulBandwidth);
end

function writeScenario(file, P, den, corners, pos)
% Translate everything by P.shift, then write header + one data row
n = size(pos, 1);
P.domainMin = P.domainMin + P.shift;
P.domainMax = P.domainMax + P.shift;
mu = den.mu + P.shift(1:2);
corners = corners + [P.shift, P.shift];
pos = pos + P.shift;
c = den.cov;
objVar = reshape([c(:, 1), c(:, 3), c(:, 3), c(:, 2)]', 1, []); % [v11, v12, v21, v22] per Gaussian
obsMin = reshape(corners(:, 1:3)', 1, []);
obsMax = reshape(corners(:, 4:6)', 1, []);
cols = {
    "timestep",                 P.timestep
    "maxIter",                  P.maxIter
    "minAlt",                   P.minAlt
    "discretizationStep",       P.gridSpacing
    "protectedRange",           P.protectedRange
    "initialStepSize",          P.initialStepSize
    "barrierGain",              P.barrierGain
    "barrierExponent",          P.barrierExponent
    "collisionRadius",          P.safetyRadius * ones(1, n)
    "comRange",                 P.comRange
    "alphaDist",                P.alphaDist
    "betaDist",                 P.betaDist
    "alphaTilt",                P.alphaTilt
    "betaTilt",                 P.betaTilt
    "domainMin",                P.domainMin
    "domainMax",                P.domainMax
    "objectivePos",             reshape(mu', 1, [])
    "objectiveVar",             objVar
    "sensorPerformanceMinimum", P.partitionMin
    "initialPositions",         reshape(pos', 1, [])
    "numObstacles",             size(corners, 1)
    "obstacleMin",              obsMin
    "obstacleMax",              obsMax
    "useDoubleIntegrator",      P.useDoubleIntegrator
    "dampingCoeff",             P.dampingCoeff
    "useFixedTopology",         P.useFixedTopology
    "txPower",                  P.backhaulTxPower * ones(1, n)
    "useSinrComms",             P.useSinrComms
    "sinrThreshold",            P.sinrThreshold
    "pathLossExponent",         P.backhaulLossExponent
    "ambientTemp",              P.backhaulTemp
    "centerFreq",               P.backhaulFreq
    "bandwidth",                P.backhaulBandwidth
    "sensorType",               1
    "serviceTxPower",           P.serviceTxPower
    "beamwidthExponent",        P.beamwidthExponent
    "serviceTilt",              P.serviceTilt
    "serviceAzimuth",           P.serviceAzimuth
    "serviceRxGain",            P.serviceRxGain
    "serviceLossExponent",      P.serviceLossExponent
    "serviceTemp",              P.serviceTemp
    "serviceCenterFreq",        P.serviceFreq
    "serviceBandwidth",         P.serviceBandwidth
    };
fields = strings(1, size(cols, 1));
for kk = 1:size(cols, 1)
    v = cols{kk, 2};
    txt = strjoin(arrayfun(@(x) string(sprintf("%.10g", x)), v), ", ");
    if numel(v) > 1
        txt = """" + txt + """";
    end
    fields(kk) = txt;
end
fid = fopen(file, "w");
assert(fid > 0, "Could not open %s for writing", file);
closer = onCleanup(@() fclose(fid));
fprintf(fid, "%s\n", strjoin(string(cols(:, 1))', ", "));
fprintf(fid, "%s\n", strjoin(fields, ", "));
end
