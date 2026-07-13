function obj = initializeFromCsv(obj, csvPath)
% INITIALIZEFROMCSV  Initialize miSim from an AERPAW scenario CSV file.
%
% Reads all guidance parameters, domain geometry, initial UAV positions,
% and obstacle definitions from the CSV, then builds and initialises the
% simulation. Ends by calling the standard obj.initialize(...) method.
%
% This is the MATLAB-path counterpart to the compiled path that unpacks a
% flat scenarioParams array in guidance_step.m.  It is only ever called
% from within a coder.target('MATLAB') guard and is never compiled.
%
% Usage (inside guidance_step.m on MATLAB path):
%   sim = sim.initializeFromCsv('aerpaw/config/scenario.csv');
%
% Expected CSV columns (see scenario.csv):
%   timestep, maxIter, minAlt, discretizationStep, protectedRange,
%   initialStepSize, barrierGain, barrierExponent,
%   collisionRadius (per-UAV), comRange (per-UAV),
%   alphaDist (per-UAV), betaDist (per-UAV),
%   alphaTilt (per-UAV), betaTilt (per-UAV),
%   domainMin ("x,y,z"), domainMax ("x,y,z"), objectivePos ("x,y"),
%   objectiveVar ("v11,v12,v21,v22"), sensorPerformanceMinimum,
%   initialPositions (flat "x1,y1,z1, x2,y2,z2,..."),
%   numObstacles, obstacleMin (flat), obstacleMax (flat)

arguments (Input)
    obj     (1, 1) {mustBeA(obj, 'miSim')};
    csvPath (1, 1) string;
end
arguments (Output)
    obj (1, 1) {mustBeA(obj, 'miSim')};
end

% ---- Parse CSV via readScenarioCsv ---------------------------------------
scenario = obj.readScenarioCsv(csvPath);

TIMESTEP            = scenario.timestep;
MAX_ITER            = scenario.maxIter;
MIN_ALT             = scenario.minAlt;
DISCRETIZATION_STEP = scenario.discretizationStep;
PROTECTED_RANGE     = scenario.protectedRange;
INITIAL_STEP_SIZE   = scenario.initialStepSize;
BARRIER_GAIN        = scenario.barrierGain;
BARRIER_EXPONENT    = scenario.barrierExponent;
% Per-UAV parameters (vectors with one element per UAV)
COLLISION_RADIUS_VEC = scenario.collisionRadius;   % 1×N
COMMS_RANGE_VEC      = scenario.comRange;           % 1×N
ALPHA_DIST_VEC       = scenario.alphaDist;          % 1×N
BETA_DIST_VEC        = scenario.betaDist;           % 1×N
ALPHA_TILT_VEC       = scenario.alphaTilt;          % 1×N
BETA_TILT_VEC        = scenario.betaTilt;           % 1×N

DOMAIN_MIN                 = scenario.domainMin;                % 1×3
DOMAIN_MAX                 = scenario.domainMax;                % 1×3

% objectivePos: 2 values per Gaussian component (1 or 2 components supported)
nObjComponents = numel(scenario.objectivePos) / 2;
assert(mod(numel(scenario.objectivePos), 2) == 0, ...
    'objectivePos must have an even number of values (2 per Gaussian component)');
assert(nObjComponents >= 1 && nObjComponents <= 2, ...
    'At most 2 objective Gaussian components supported; got %d', nObjComponents);
assert(numel(scenario.objectiveVar) == nObjComponents * 4, ...
    'objectiveVar must have %d values for %d component(s); got %d', ...
    nObjComponents * 4, nObjComponents, numel(scenario.objectiveVar));
OBJECTIVE_GROUND_POS = reshape(scenario.objectivePos, 2, nObjComponents)';           % nObj×2
OBJECTIVE_VAR        = permute(reshape(scenario.objectiveVar, 2, 2, nObjComponents), [3, 1, 2]); % nObj×2×2

SENSOR_PERFORMANCE_MINIMUM = scenario.sensorPerformanceMinimum; % scalar

% Initial UAV positions: flat vector reshaped to N×3
flatPos = scenario.initialPositions;          % 1×(3*N)
assert(mod(numel(flatPos), 3) == 0, ...
    "initialPositions must have a multiple of 3 values; got %d", numel(flatPos));
positions  = reshape(flatPos, 3, [])';        % N×3
numAgents  = size(positions, 1);

% Validate per-UAV parameter lengths match numAgents
assert(numel(COLLISION_RADIUS_VEC) == numAgents, ...
    "collisionRadius has %d values but expected %d (one per UAV)", numel(COLLISION_RADIUS_VEC), numAgents);
assert(numel(COMMS_RANGE_VEC) == numAgents, ...
    "comRange has %d values but expected %d (one per UAV)", numel(COMMS_RANGE_VEC), numAgents);
assert(numel(ALPHA_DIST_VEC) == numAgents, ...
    "alphaDist has %d values but expected %d (one per UAV)", numel(ALPHA_DIST_VEC), numAgents);
assert(numel(BETA_DIST_VEC) == numAgents, ...
    "betaDist has %d values but expected %d (one per UAV)", numel(BETA_DIST_VEC), numAgents);
assert(numel(ALPHA_TILT_VEC) == numAgents, ...
    "alphaTilt has %d values but expected %d (one per UAV)", numel(ALPHA_TILT_VEC), numAgents);
assert(numel(BETA_TILT_VEC) == numAgents, ...
    "betaTilt has %d values but expected %d (one per UAV)", numel(BETA_TILT_VEC), numAgents);

numObstacles = scenario.numObstacles;

% Dynamics model (optional columns — backward compatible with older CSVs)
if isfield(scenario, 'useDoubleIntegrator')
    USE_DOUBLE_INTEGRATOR = logical(scenario.useDoubleIntegrator);
else
    USE_DOUBLE_INTEGRATOR = false;
end
if isfield(scenario, 'dampingCoeff')
    DAMPING_COEFF = scenario.dampingCoeff;
else
    DAMPING_COEFF = 2.0;
end
if isfield(scenario, 'useFixedTopology')
    USE_FIXED_TOPOLOGY = logical(scenario.useFixedTopology);
else
    USE_FIXED_TOPOLOGY = false;
end

% SINR communications model (optional columns — backward compatible with
% older CSVs that predate the SINR model; defaults keep the fixed-radius path)
if isfield(scenario, 'txPower')
    TX_POWER_VEC = scenario.txPower;                       % 1×N (Watts)
    assert(numel(TX_POWER_VEC) == numAgents, ...
        "txPower has %d values but expected %d (one per UAV)", numel(TX_POWER_VEC), numAgents);
else
    TX_POWER_VEC = 0.1 * ones(1, numAgents);
end
if isfield(scenario, 'useSinrComms')
    USE_SINR_COMMS = logical(scenario.useSinrComms);
else
    USE_SINR_COMMS = false;
end
if isfield(scenario, 'sinrThreshold')
    SINR_THRESHOLD = scenario.sinrThreshold;               % dB
else
    SINR_THRESHOLD = 0;
end
if isfield(scenario, 'pathLossExponent')
    PATH_LOSS_EXPONENT = scenario.pathLossExponent;
else
    PATH_LOSS_EXPONENT = 2.0;
end
if isfield(scenario, 'ambientTemp')
    AMBIENT_TEMP = scenario.ambientTemp;                   % Kelvin
else
    AMBIENT_TEMP = 290.0;
end
if isfield(scenario, 'centerFreq')
    CENTER_FREQ = scenario.centerFreq;                     % Hz
else
    CENTER_FREQ = 2.4e9;
end
if isfield(scenario, 'bandwidth')
    BANDWIDTH = scenario.bandwidth;                        % Hz
else
    BANDWIDTH = 20e6;
end

% Routing topology selection (optional columns; MATLAB sim only)
if isfield(scenario, 'useRoutingTopology')
    USE_ROUTING_TOPOLOGY = logical(scenario.useRoutingTopology);
else
    USE_ROUTING_TOPOLOGY = false;
end
if isfield(scenario, 'routingFlowThreshold')
    ROUTING_FLOW_THRESHOLD = scenario.routingFlowThreshold;
else
    ROUTING_FLOW_THRESHOLD = 0.02;
end

% ---- Build domain --------------------------------------------------------
dom = rectangularPrism;
dom = dom.initialize([DOMAIN_MIN; DOMAIN_MAX], REGION_TYPE.DOMAIN, "Guidance Domain");

% ---- Build sensing objective (MATLAB path: objectiveFunctionWrapper) -----
dom.objective = sensingObjective;
objFcn = objectiveFunctionWrapper(OBJECTIVE_GROUND_POS, OBJECTIVE_VAR);
dom.objective = dom.objective.initialize(objFcn, dom, DISCRETIZATION_STEP, PROTECTED_RANGE, SENSOR_PERFORMANCE_MINIMUM);

% ---- Initialise agents from scenario positions ---------------------------
% Each agent gets its own sensor model and collision/comms radii from the
% per-UAV parameter vectors.
agentList = cell(numAgents, 1);
for ii = 1:numAgents
    pos  = positions(ii, :);

    % Per-UAV sensor model
    sensor = sigmoidSensor;
    sensor = sensor.initialize(ALPHA_DIST_VEC(ii), BETA_DIST_VEC(ii), ...
                               ALPHA_TILT_VEC(ii), BETA_TILT_VEC(ii));

    geom = spherical;
    geom = geom.initialize(pos, COLLISION_RADIUS_VEC(ii), REGION_TYPE.COLLISION, ...
                           sprintf("UAV %d Collision", ii));
    ag = agent;
    ag = ag.initialize(pos, geom, sensor, COMMS_RANGE_VEC(ii), MAX_ITER, ...
                       INITIAL_STEP_SIZE, sprintf("UAV %d", ii));
    ag.txPower = TX_POWER_VEC(ii); % SINR comms model transmit power (W)
    agentList{ii} = ag;
end

% ---- Build obstacles from CSV --------------------------------------------
obstacleList = cell(numObstacles, 1);
if numObstacles > 0
    obsMin = reshape(scenario.obstacleMin, 3, numObstacles)';  % N×3
    obsMax = reshape(scenario.obstacleMax, 3, numObstacles)';
    for ii = 1:numObstacles
        obs = rectangularPrism;
        obs = obs.initialize([obsMin(ii, :); obsMax(ii, :)], ...
                             REGION_TYPE.OBSTACLE, sprintf("Obstacle %d", ii));
        obstacleList{ii} = obs;
    end
end

% ---- Initialise simulation (plots and video disabled) --------------------
obj = obj.initialize(dom, agentList, BARRIER_GAIN, BARRIER_EXPONENT, ...
                     MIN_ALT, TIMESTEP, MAX_ITER, obstacleList, false, false, ...
                     USE_DOUBLE_INTEGRATOR, DAMPING_COEFF, USE_FIXED_TOPOLOGY, false, ...
                     USE_SINR_COMMS, SINR_THRESHOLD, PATH_LOSS_EXPONENT, AMBIENT_TEMP, CENTER_FREQ, BANDWIDTH, ...
                     USE_ROUTING_TOPOLOGY, ROUTING_FLOW_THRESHOLD);

end
