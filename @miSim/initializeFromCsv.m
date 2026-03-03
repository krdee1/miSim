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
%   initialStepSize, barrierGain, barrierExponent, collisionRadius, comRange,
%   alphaDist, betaDist, alphaTilt, betaTilt,
%   domainMin ("x,y,z"), domainMax ("x,y,z"), objectivePos ("x,y"),
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
COLLISION_RADIUS    = scenario.collisionRadius;
COMMS_RANGE         = scenario.comRange;
ALPHA_DIST          = scenario.alphaDist;
BETA_DIST           = scenario.betaDist;
ALPHA_TILT          = scenario.alphaTilt;
BETA_TILT           = scenario.betaTilt;

DOMAIN_MIN                 = scenario.domainMin;                % 1×3
DOMAIN_MAX                 = scenario.domainMax;                % 1×3
OBJECTIVE_GROUND_POS       = scenario.objectivePos;             % 1×2
OBJECTIVE_VAR              = reshape(scenario.objectiveVar, 2, 2); % 2×2 covariance matrix
SENSOR_PERFORMANCE_MINIMUM = scenario.sensorPerformanceMinimum; % scalar

% Initial UAV positions: flat vector reshaped to N×3
flatPos = scenario.initialPositions;          % 1×(3*N)
assert(mod(numel(flatPos), 3) == 0, ...
    "initialPositions must have a multiple of 3 values; got %d", numel(flatPos));
positions  = reshape(flatPos, 3, [])';        % N×3
numAgents  = size(positions, 1);

numObstacles = scenario.numObstacles;

% ---- Build domain --------------------------------------------------------
dom = rectangularPrism;
dom = dom.initialize([DOMAIN_MIN; DOMAIN_MAX], REGION_TYPE.DOMAIN, "Guidance Domain");

% ---- Build sensing objective (MATLAB path: objectiveFunctionWrapper) -----
dom.objective = sensingObjective;
objFcn = objectiveFunctionWrapper(OBJECTIVE_GROUND_POS, OBJECTIVE_VAR);
dom.objective = dom.objective.initialize(objFcn, dom, DISCRETIZATION_STEP, PROTECTED_RANGE, SENSOR_PERFORMANCE_MINIMUM);

% ---- Build shared sensor model -------------------------------------------
sensor = sigmoidSensor;
sensor = sensor.initialize(ALPHA_DIST, BETA_DIST, ALPHA_TILT, BETA_TILT);

% ---- Initialise agents from scenario positions ---------------------------
agentList = cell(numAgents, 1);
for ii = 1:numAgents
    pos  = positions(ii, :);
    geom = spherical;
    geom = geom.initialize(pos, COLLISION_RADIUS, REGION_TYPE.COLLISION, ...
                           sprintf("UAV %d Collision", ii));
    ag = agent;
    ag = ag.initialize(pos, geom, sensor, COMMS_RANGE, MAX_ITER, ...
                       INITIAL_STEP_SIZE, sprintf("UAV %d", ii));
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
                     MIN_ALT, TIMESTEP, MAX_ITER, obstacleList, false, false);

end
