function nextPositions = guidance_step(currentPositions, isInit)
% guidance_step  One step of the miSim sensing coverage guidance algorithm.
%
% Wraps the miSim gradient-ascent + CBF motion algorithm for AERPAW.
% Holds full simulation state in a persistent variable between calls.
%
% Usage (from controller.m):
%   guidance_step(initPositions, true)         % first call: initialise state
%   nextPos = guidance_step(gpsPos, false)     % subsequent calls: run one step
%
% Inputs:
%   currentPositions  (MAX_CLIENTS × 3) double  ENU [east north up] metres
%   isInit            (1,1)             logical  true on first call only
%
% Output:
%   nextPositions     (MAX_CLIENTS × 3) double  guidance targets, ENU metres
%
% Codegen notes:
%   - Persistent variable 'sim' holds the miSim object between calls.
%   - Plotting/video are disabled (makePlots=false, makeVideo=false) for
%     deployment. The coder.target('MATLAB') guards in the miSim/agent
%     class files must be in place before codegen will succeed.
%   - objectiveFunctionWrapper returns a function handle which is not
%     directly codegen-compatible; the MATLAB path uses it normally. The
%     compiled path requires an equivalent C impl (see TODO below).

coder.extrinsic('disp', 'objectiveFunctionWrapper');

% =========================================================================
% Tunable guidance parameters — adjust here and recompile as needed.
% =========================================================================
MAX_CLIENTS = int32(4);      % must match MAX_CLIENTS in controller.m

% Domain bounds in ENU metres [east, north, up]
DOMAIN_MIN = [  0.0,   0.0,  25.0];
DOMAIN_MAX = [ 20.0,  20.0,  55.0];
MIN_ALT    = 25.0;           % hard altitude floor (m)

% Sensing objective: bivariate Gaussian centred at [east, north]
OBJECTIVE_GROUND_POS = [10.0, 10.0];
DISCRETIZATION_STEP  = 0.1;  % objective grid step (m) — coarser = faster
PROTECTED_RANGE      = 1.0;  % objective centre must be >this from domain edge

% Agent safety geometry
COLLISION_RADIUS = 3.0;      % spherical collision radius (m)
COMMS_RANGE      = 60.0;     % communications range (m)

% Gradient-ascent parameters
INITIAL_STEP_SIZE = 1;     % step size at iteration 0 (decays to 0 at MAX_ITER)
MAX_ITER          = 100;     % guidance steps (sets decay rate)

% Sensor model (sigmoidSensor)
ALPHA_DIST = 60.0;           % effective sensing distance (m) — set beyond max domain slant range (~53 m)
BETA_DIST  = 0.2;            % distance sigmoid steepness — gentle, tilt drives the coverage gradient
ALPHA_TILT = 10.0;           % max useful tilt angle (degrees)
BETA_TILT  = 1.0;            % tilt sigmoid steepness

% Safety filter — Control Barrier Function (CBF/QP)
BARRIER_GAIN     = 100;
BARRIER_EXPONENT = 3;

% Simulation timestep (s) — should match GUIDANCE_RATE_MS / 1000 in controller.m
TIMESTEP = 5.0;
% =========================================================================

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
    end

    % --- Build domain geometry ---
    dom = rectangularPrism;
    dom = dom.initialize([DOMAIN_MIN; DOMAIN_MAX], REGION_TYPE.DOMAIN, "Guidance Domain");

    % --- Build sensing objective ---
    dom.objective = sensingObjective;
    if coder.target('MATLAB')
        % objectiveFunctionWrapper returns a function handle — MATLAB only.
        objFcn = objectiveFunctionWrapper(OBJECTIVE_GROUND_POS, 3*eye(2));
        dom.objective = dom.objective.initialize(objFcn, dom, ...
            DISCRETIZATION_STEP, PROTECTED_RANGE);
    else
        % Evaluate bivariate Gaussian inline (codegen-compatible; no function handle).
        % Must build the same grid that initializeWithValues uses internally.
        xGrid = unique([DOMAIN_MIN(1):DISCRETIZATION_STEP:DOMAIN_MAX(1), DOMAIN_MAX(1)]);
        yGrid = unique([DOMAIN_MIN(2):DISCRETIZATION_STEP:DOMAIN_MAX(2), DOMAIN_MAX(2)]);
        [gridX, gridY] = meshgrid(xGrid, yGrid);
        dx = gridX - OBJECTIVE_GROUND_POS(1);
        dy = gridY - OBJECTIVE_GROUND_POS(2);
        objValues = exp(-0.5 * (dx .* dx + dy .* dy));
        dom.objective = dom.objective.initializeWithValues(objValues, dom, ...
            DISCRETIZATION_STEP, PROTECTED_RANGE);
    end

    % --- Build shared sensor model ---
    sensor = sigmoidSensor;
    sensor = sensor.initialize(ALPHA_DIST, BETA_DIST, ALPHA_TILT, BETA_TILT);

    % --- Initialise agents from GPS positions ---
    agentList = cell(numAgents, 1);
    for ii = 1:numAgents
        pos  = currentPositions(ii, :);
        geom = spherical;
        geom = geom.initialize(pos, COLLISION_RADIUS, REGION_TYPE.COLLISION, ...
                               sprintf("UAV %d Collision", ii));
        ag = agent;
        ag = ag.initialize(pos, geom, sensor, COMMS_RANGE, MAX_ITER, ...
                           INITIAL_STEP_SIZE, sprintf("UAV %d", ii));
        agentList{ii} = ag;
    end

    % --- Initialise simulation (plots and video disabled) ---
    sim = miSim;
    sim = sim.initialize(dom, agentList, BARRIER_GAIN, BARRIER_EXPONENT, ...
                         MIN_ALT, TIMESTEP, MAX_ITER, cell(0, 1), false, false);

    if coder.target('MATLAB')
        disp('[guidance_step] Initialisation complete.');
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
    sim = sim.lesserNeighbor();

    % 4. Compute Voronoi partitioning
    sim.partitioning = sim.agents{1}.partition(sim.agents, sim.domain.objective);

    % 5. Unconstrained gradient-ascent step for each agent
    for ii = 1:size(sim.agents, 1)
        sim.agents{ii} = sim.agents{ii}.run(sim.domain, sim.partitioning, ...
                                             sim.timestepIndex, ii, sim.agents);
    end

    % 6. Apply CBF safety filter (collision / comms / domain constraints via QP)
    sim = sim.constrainMotion();

    % 7. Return constrained next positions
    for ii = 1:size(sim.agents, 1)
        nextPositions(ii, :) = sim.agents{ii}.pos;
    end

end

end
