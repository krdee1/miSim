function obj = initializeFromInits(obj, initsPath)
% INITIALIZEFROMINITS  Initialize miSim from a saved simInits matfile.
%
% Loads all simulation parameters and initial agent states written by
% writeInits(), reconstructs domain, objective, agents, and obstacles, then
% calls the standard obj.initialize() method.  Plots and video are disabled.
%
% Usage:
%   sim = sim.initializeFromInits('sandbox/2025_01_01_12_00_00_miSimInits.mat');

arguments (Input)
    obj       (1, 1) {mustBeA(obj, 'miSim')};
    initsPath (1, 1) string;
end
arguments (Output)
    obj (1, 1) {mustBeA(obj, 'miSim')};
end

inits = load(initsPath);

% ---- Build domain ------------------------------------------------------------
dom = rectangularPrism;
dom = dom.initialize([inits.domainMin; inits.domainMax], REGION_TYPE.DOMAIN, "Domain");

% ---- Build sensing objective -------------------------------------------------
dom.objective = sensingObjective;
% reshape guards against MATLAB flattening the 1×2×2 singleton dimension on load
objSigma = reshape(inits.objectiveSigma, [1 2 2]);
objFcn = objectiveFunctionWrapper(inits.objectivePos, objSigma);
dom.objective = dom.objective.initialize(objFcn, dom, ...
    inits.discretizationStep, inits.protectedRange, inits.sensorPerformanceMinimum, ...
    inits.objectivePos, objSigma);

% ---- Build agents ------------------------------------------------------------
numAgents = inits.numAgents;
agentList = cell(numAgents, 1);
for ii = 1:numAgents
    pos = inits.pos(ii, :);

    sensor = sigmoidSensor;
    sensor = sensor.initialize(inits.alphaDist(ii), inits.betaDist(ii), ...
                               inits.alphaTilt(ii), inits.betaTilt(ii));

    geom = spherical;
    geom = geom.initialize(pos, inits.collisionRadius(ii), REGION_TYPE.COLLISION, ...
                           sprintf("UAV %d Collision", ii));
    ag = agent;
    ag = ag.initialize(pos, geom, sensor, inits.comRange(ii), inits.maxIter, ...
                       inits.initialStepSize(ii), sprintf("UAV %d", ii));
    agentList{ii} = ag;
end

% ---- Build obstacles ---------------------------------------------------------
numObstacles = inits.numObstacles;
obstacleList = cell(numObstacles, 1);
if numObstacles > 0
    for ii = 1:numObstacles
        obs = rectangularPrism;
        obs = obs.initialize([inits.obsMinCorners(ii, :); inits.obsMaxCorners(ii, :)], ...
                             REGION_TYPE.OBSTACLE, sprintf("Obstacle %d", ii));
        obstacleList{ii} = obs;
    end
end

% ---- Optional backward-compat fields -----------------------------------------
if isfield(inits, 'useDoubleIntegrator')
    useDoubleIntegrator = logical(inits.useDoubleIntegrator);
else
    useDoubleIntegrator = false;
end
if isfield(inits, 'dampingCoeff')
    dampingCoeff = inits.dampingCoeff;
else
    dampingCoeff = 2.0;
end
if isfield(inits, 'useFixedTopology')
    useFixedTopology = logical(inits.useFixedTopology);
else
    useFixedTopology = false;
end

% ---- Initialize simulation (plots and video disabled) ------------------------
obj = obj.initialize(dom, agentList, inits.barrierGain, inits.barrierExponent, ...
                     inits.minAlt, inits.timestep, inits.maxIter, obstacleList, ...
                     false, false, useDoubleIntegrator, dampingCoeff, useFixedTopology);

end
