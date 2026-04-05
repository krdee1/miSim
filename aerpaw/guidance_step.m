function nextPositions = guidance_step(currentPositions, isInit, ...
                                       scenarioParams, ...
                                       obstacleMin, obstacleMax, numObstacles)
% guidance_step  One step of the miSim sensing coverage guidance algorithm.
%
% Wraps the miSim gradient-ascent + CBF motion algorithm for AERPAW.
% Holds full simulation state in a persistent variable between calls.
%
% Usage (from controller.m):
%   guidance_step(initPositions, true,  scenarioParams, obstacleMin, obstacleMax, numObstacles)
%   nextPos = guidance_step(gpsPos, false, scenarioParams, obstacleMin, obstacleMax, numObstacles)
%
% Inputs:
%   currentPositions  (MAX_CLIENTS × 3) double  ENU [east north up] metres
%   isInit            (1,1)             logical  true on first call only
%   scenarioParams    (1 × NUM_SCENARIO_PARAMS) double
%                       Flat array of guidance parameters (compiled path).
%                       On MATLAB path this is ignored; parameters are loaded
%                       from scenario.csv via initializeFromCsv instead.
%                       Index mapping (1-based):
%                         1  timestep            9-12  collisionRadius[1:4]
%                         2  maxIter            13-16  comRange[1:4]
%                         3  minAlt             17-20  alphaDist[1:4]
%                         4  discretizationStep 21-24  betaDist[1:4]
%                         5  protectedRange     25-28  alphaTilt[1:4]
%                         6  initialStepSize    29-32  betaTilt[1:4]
%                         7  barrierGain        33-35  domainMin
%                         8  barrierExponent    36-38  domainMax
%                                               39-40  objectivePos
%                                               41-44  objectiveVar (2x2, col-major)
%                                               45     sensorPerformanceMinimum
%                                               46     useDoubleIntegrator
%                                               47     dampingCoeff
%                                               48     useFixedTopology
%   obstacleMin       (MAX_OBSTACLES × 3) double  column-major obstacle corners (compiled path)
%   obstacleMax       (MAX_OBSTACLES × 3) double
%   numObstacles      (1,1) int32                 actual obstacle count
%
% Output:
%   nextPositions     (MAX_CLIENTS × 3) double  guidance targets, ENU metres
%
% Codegen notes:
%   - Persistent variable 'sim' holds the miSim object between calls.
%   - On MATLAB path isInit uses sim.initializeFromCsv (file I/O, not compiled).
%   - On compiled path isInit uses scenarioParams + obstacle arrays directly.
%   - Plotting/video are disabled (makePlots=false, makeVideo=false).

coder.extrinsic('disp', 'objectiveFunctionWrapper', 'initializeFromCsv');

MAX_CLIENTS = int32(4);      % must match MAX_CLIENTS in controller.m

% Path to scenario CSV — used on MATLAB path only (not compiled)
SCENARIO_CSV = 'aerpaw/config/scenario.csv';

persistent sim;
if isempty(sim)
    sim = miSim;
end

% Pre-allocate output with known static size (required for codegen)
nextPositions = zeros(MAX_CLIENTS, 3);

numAgents = int32(size(currentPositions, 1));

if isInit
    if coder.target('MATLAB')
        disp('[guidance_step] Initialising simulation...');

        % MATLAB path: load all parameters and obstacles from scenario CSV.
        % initializeFromCsv reads the file, builds domain/agents/obstacles,
        % and calls sim.initialize internally.
        sim = sim.initializeFromCsv(SCENARIO_CSV);

        disp('[guidance_step] Initialisation complete.');
    else
        % ================================================================
        % Compiled path: unpack scenarioParams array and obstacle arrays.
        % Per-UAV parameters are stored as MAX_CLIENTS-wide blocks; only
        % the first numAgents entries of each block are used.
        % ================================================================
        TIMESTEP            = scenarioParams(1);
        MAX_ITER            = int32(scenarioParams(2));
        MIN_ALT             = scenarioParams(3);
        DISCRETIZATION_STEP = scenarioParams(4);
        PROTECTED_RANGE     = scenarioParams(5);
        INITIAL_STEP_SIZE   = scenarioParams(6);
        BARRIER_GAIN        = scenarioParams(7);
        BARRIER_EXPONENT    = scenarioParams(8);
        COLLISION_RADIUS_VEC = scenarioParams(9:12);   % per-UAV [1:MAX_CLIENTS]
        COMMS_RANGE_VEC      = scenarioParams(13:16);
        ALPHA_DIST_VEC       = scenarioParams(17:20);
        BETA_DIST_VEC        = scenarioParams(21:24);
        ALPHA_TILT_VEC       = scenarioParams(25:28);
        BETA_TILT_VEC        = scenarioParams(29:32);
        DOMAIN_MIN                  = scenarioParams(33:35);
        DOMAIN_MAX                  = scenarioParams(36:38);
        OBJECTIVE_GROUND_POS        = scenarioParams(39:40);
        OBJECTIVE_VAR               = reshape(scenarioParams(41:44), 2, 2);
        SENSOR_PERFORMANCE_MINIMUM  = scenarioParams(45);
        USE_DOUBLE_INTEGRATOR       = logical(scenarioParams(46));
        DAMPING_COEFF               = scenarioParams(47);
        USE_FIXED_TOPOLOGY          = logical(scenarioParams(48));

        % --- Build domain geometry ---
        dom = rectangularPrism;
        dom = dom.initialize([DOMAIN_MIN; DOMAIN_MAX], REGION_TYPE.DOMAIN, "Guidance Domain");

        % --- Build sensing objective (inline Gaussian; codegen-compatible) ---
        dom.objective = sensingObjective;
        xGrid = unique([DOMAIN_MIN(1):DISCRETIZATION_STEP:DOMAIN_MAX(1), DOMAIN_MAX(1)]);
        yGrid = unique([DOMAIN_MIN(2):DISCRETIZATION_STEP:DOMAIN_MAX(2), DOMAIN_MAX(2)]);
        [gridX, gridY] = meshgrid(xGrid, yGrid);
        dx = gridX - OBJECTIVE_GROUND_POS(1);
        dy = gridY - OBJECTIVE_GROUND_POS(2);
        % Bivariate Gaussian using objectiveVar covariance matrix (avoids inv())
        ov_a = OBJECTIVE_VAR(1,1); ov_b = OBJECTIVE_VAR(1,2);
        ov_c = OBJECTIVE_VAR(2,1); ov_d = OBJECTIVE_VAR(2,2);
        ov_det = ov_a * ov_d - ov_b * ov_c;
        objValues = exp((-0.5 / ov_det) .* (ov_d .* dx.*dx - (ov_b + ov_c) .* dx.*dy + ov_a .* dy.*dy));
        dom.objective = dom.objective.initializeWithValues(objValues, dom, ...
            DISCRETIZATION_STEP, PROTECTED_RANGE, SENSOR_PERFORMANCE_MINIMUM);

        % --- Initialise agents from GPS positions (per-UAV parameters) ----
        agentList = cell(numAgents, 1);
        for ii = 1:numAgents
            pos  = currentPositions(ii, :);

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
            agentList{ii} = ag;
        end

        % --- Build obstacle list from flat arrays ---
        coder.varsize('obstacleList', [8, 1], [1, 0]);
        obstacleList = repmat({rectangularPrism}, numObstacles, 1);
        for ii = 1:numObstacles
            obs = rectangularPrism;
            obs = obs.initialize([obstacleMin(ii, :); obstacleMax(ii, :)], ...
                                 REGION_TYPE.OBSTACLE, sprintf("Obstacle %d", ii));
            obstacleList{ii} = obs;
        end

        % --- Initialise simulation (plots and video disabled) ---
        sim = miSim;
        sim = sim.initialize(dom, agentList, BARRIER_GAIN, BARRIER_EXPONENT, ...
                             MIN_ALT, TIMESTEP, MAX_ITER, obstacleList, false, false, ...
                             USE_DOUBLE_INTEGRATOR, DAMPING_COEFF, USE_FIXED_TOPOLOGY);
    end

    % On the init call return current positions unchanged
    for ii = 1:numAgents
        nextPositions(ii, :) = currentPositions(ii, :);
    end

else
    % =====================================================================
    % One guidance step
    % =====================================================================

    % 1. Inject actual GPS positions (closed-loop feedback)
    for ii = 1:size(sim.agents, 1)
        sim.agents{ii}.lastPos = sim.agents{ii}.pos;
        sim.agents{ii}.pos     = currentPositions(ii, :);

        % Re-centre collision geometry at new position
        d = currentPositions(ii, :) - sim.agents{ii}.collisionGeometry.center;
        sim.agents{ii}.collisionGeometry = sim.agents{ii}.collisionGeometry.initialize( ...
            sim.agents{ii}.collisionGeometry.center + d, ...
            sim.agents{ii}.collisionGeometry.radius, ...
            REGION_TYPE.COLLISION);
    end

    % 2. Advance timestep counter
    sim.timestepIndex = sim.timestepIndex + 1;

    % 3. Update communications topology (Lesser Neighbour Assignment)
    if ~sim.useFixedTopology
        sim = sim.lesserNeighbor();
    end

    % 4. Compute Voronoi partitioning
    sim.partitioning = sim.agents{1}.partition(sim.agents, sim.domain.objective);

    % 5. Unconstrained gradient-ascent step for each agent
    for ii = 1:size(sim.agents, 1)
        sim.agents{ii} = sim.agents{ii}.run(sim.domain, sim.partitioning, ...
                                             sim.timestepIndex, ii, sim.agents, ...
                                             sim.useDoubleIntegrator, sim.dampingCoeff, sim.timestep);
    end

    % 6. Apply CBF safety filter (collision / comms / domain constraints via QP)
    sim = sim.constrainMotion();

    % 7. Return constrained next positions
    for ii = 1:size(sim.agents, 1)
        nextPositions(ii, :) = sim.agents{ii}.pos;
    end

end

end
