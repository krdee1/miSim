function controller(numClients)
arguments (Input)
    numClients (1, 1) int32;
end

coder.extrinsic('disp', 'readScenarioCsv');

% Maximum clients supported (one initial position per UAV)
MAX_CLIENTS = 4;
MAX_TARGETS = MAX_CLIENTS;

% Allocate targets array (MAX_TARGETS x 3)
targets = zeros(MAX_TARGETS, 3);
numWaypoints = int32(0);
totalLoaded  = int32(0);  % pre-declare type for coder.ceval %#ok<NASGU>

% Load initial UAV positions from scenario CSV
if coder.target('MATLAB')
    disp('Loading initial positions from scenario.csv (simulation)...');
    sc = readScenarioCsv('aerpaw/config/scenario.csv');
    flatPos = double(sc.initialPositions);  % 1×(3*N) flat vector
    posMatrix = reshape(flatPos, 3, [])';   % N×3, same layout as initializeFromCsv
    totalLoaded = int32(size(posMatrix, 1));
    targets(1:totalLoaded, :) = posMatrix;
    numWaypoints = int32(1);
    disp(['Loaded ', num2str(double(totalLoaded)), ' initial positions']);
else
    coder.cinclude('controller_impl.h');
    filename = ['config/scenario.csv', char(0)];
    totalLoaded = coder.ceval('loadInitialPositions', coder.ref(filename), ...
                coder.ref(targets), int32(MAX_TARGETS));
    numWaypoints = totalLoaded / int32(numClients);
end

% Load guidance scenario from CSV (parameters for guidance_step)
NUM_SCENARIO_PARAMS = 21;
MAX_OBSTACLES_CTRL  = int32(8);
scenarioParams = zeros(1, NUM_SCENARIO_PARAMS);
obstacleMin    = zeros(MAX_OBSTACLES_CTRL, 3);
obstacleMax    = zeros(MAX_OBSTACLES_CTRL, 3);
numObstacles   = int32(0);
perAgentParams = zeros(MAX_CLIENTS, 6);
numAgentsLoaded = int32(0);
if ~coder.target('MATLAB')
    coder.cinclude('controller_impl.h');
    scenarioFilename = ['config/scenario.csv', char(0)];
    coder.ceval('loadScenario', coder.ref(scenarioFilename), coder.ref(scenarioParams));
    numObstacles = coder.ceval('loadObstacles', coder.ref(scenarioFilename), ...
                         coder.ref(obstacleMin), coder.ref(obstacleMax), ...
                         int32(MAX_OBSTACLES_CTRL));
    numAgentsLoaded = coder.ceval('loadPerAgentParams', coder.ref(scenarioFilename), ...
                         coder.ref(perAgentParams), int32(MAX_CLIENTS));
end
% On MATLAB path, populate the scalar entries used by controller.m itself
% (guidance_step reads parameters directly via initializeFromCsv and ignores them).
if coder.target('MATLAB')
    scenarioParams(1) = double(sc.timestep);
    scenarioParams(2) = double(sc.maxIter);
end

% Initialize server
if coder.target('MATLAB')
    disp('Initializing server (simulation)...');
else
    coder.ceval('initServer');
end

% Accept clients
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Accepting client ', num2str(i)]);
    else
        coder.ceval('acceptClient', int32(i));
    end
end

% Waypoint loop: send each waypoint to all clients, wait for all to arrive
for w = 1:numWaypoints
    % Send TARGET for waypoint w to each client
    for i = 1:numClients
        % Targets are grouped by client: client i's waypoints are at rows
        % (i-1)*numWaypoints+1 through i*numWaypoints
        targetIdx = (i - 1) * numWaypoints + w;
        target = targets(targetIdx, :);

        if coder.target('MATLAB')
            disp(['Sending TARGET to client ', num2str(i), ' (waypoint ', num2str(w), '): ', ...
                  num2str(target(1)), ',', num2str(target(2)), ',', num2str(target(3))]);
        else
            coder.ceval('sendTarget', int32(i), coder.ref(target));
        end
    end

    % Wait for ACK from all clients
    if coder.target('MATLAB')
        disp('Waiting for ACK from all clients...');
    else
        coder.ceval('waitForAllMessageType', int32(numClients), ...
                    int32(MESSAGE_TYPE.ACK));
    end

    % Wait for READY from all clients (all arrived at waypoint w)
    if coder.target('MATLAB')
        disp(['All UAVs arrived at waypoint ', num2str(w)]);
    else
        coder.ceval('waitForAllMessageType', int32(numClients), ...
                    int32(MESSAGE_TYPE.READY));
    end
end

% ---- Phase 2: miSim guidance loop ----------------------------------------
% Guidance parameters loaded from scenario.csv
MAX_GUIDANCE_STEPS = int32(scenarioParams(2));        % maxIter
GUIDANCE_RATE_MS   = int32(scenarioParams(1) * 1000); % timestep (s) → ms

% Brief pause before starting guidance loop
if coder.target('MATLAB')
    pause(2);
else
    coder.ceval('sleepMs', int32(2000));
end

% Enter guidance mode on all clients
if ~coder.target('MATLAB')
    coder.ceval('sendGuidanceToggle', int32(numClients));
end

% Seed initial positions from GPS (compiled) or CSV waypoints (simulation)
positions = zeros(MAX_CLIENTS, 3);
if ~coder.target('MATLAB')
    coder.ceval('sendRequestPositions', int32(numClients));
    coder.ceval('recvPositions', int32(numClients), coder.ref(positions), int32(MAX_CLIENTS));
else
    % Simulation: seed positions from CSV waypoints so agents don't start at origin
    positions(1:totalLoaded, :) = targets(1:totalLoaded, :);
end

% Initialise guidance algorithm state
guidance_step(positions(1:numClients, :), true, ...
              scenarioParams, obstacleMin, obstacleMax, numObstacles, ...
              perAgentParams, numAgentsLoaded);

% Compute and send first targets to kick off UAV movement
nextPositions = guidance_step(positions(1:numClients, :), false, ...
                               scenarioParams, obstacleMin, obstacleMax, numObstacles, ...
                               perAgentParams, numAgentsLoaded);
for i = 1:numClients
    target = nextPositions(i, :);
    if ~coder.target('MATLAB')
        coder.ceval('sendTarget', int32(i), coder.ref(target));
    else
        disp(['[guidance] initial target UAV ', num2str(i), ': ', num2str(target)]);
    end
end
lastSendTimeMs  = 0.0;
nowMs           = 0.0;
sleepDurationMs = 0.0; %#ok<NASGU>
if ~coder.target('MATLAB')
    lastSendTimeMs = coder.ceval('getTimeMs');
end
if coder.target('MATLAB')
    positions(1:numClients, :) = nextPositions(1:numClients, :);
end

% Main guidance loop: arrival-triggered with minimum rate limiting
for step = 1:MAX_GUIDANCE_STEPS
    % 1. Wait for all UAVs to signal arrival at their current target
    if ~coder.target('MATLAB')
        coder.ceval('waitForAllMessageType', int32(numClients), int32(MESSAGE_TYPE.ACK));
    else
        disp(['[guidance] step ', num2str(step), ': simulating UAV arrival']);
    end

    % 2. Request and receive current GPS positions from all UAVs
    if ~coder.target('MATLAB')
        coder.ceval('sendRequestPositions', int32(numClients));
        coder.ceval('recvPositions', int32(numClients), coder.ref(positions), int32(MAX_CLIENTS));
    end

    % 3. Compute new target positions via guidance algorithm
    nextPositions = guidance_step(positions(1:numClients, :), false, ...
                                  scenarioParams, obstacleMin, obstacleMax, numObstacles, ...
                                  perAgentParams, numAgentsLoaded);

    % 4. Enforce minimum inter-send interval (timestep from scenario.csv)
    if ~coder.target('MATLAB')
        nowMs = coder.ceval('getTimeMs');
        sleepDurationMs = lastSendTimeMs + double(GUIDANCE_RATE_MS) - nowMs;
        if sleepDurationMs > 0.0
            coder.ceval('sleepMs', int32(sleepDurationMs));
        end
    end

    % 5. Send new targets to each UAV
    for i = 1:numClients
        target = nextPositions(i, :);
        if ~coder.target('MATLAB')
            coder.ceval('sendTarget', int32(i), coder.ref(target));
        else
            disp(['[guidance] target UAV ', num2str(i), ': ', num2str(target)]);
        end
    end
    if ~coder.target('MATLAB')
        lastSendTimeMs = coder.ceval('getTimeMs');
    end

    % Simulation: advance positions to guidance outputs for closed-loop feedback
    if coder.target('MATLAB')
        positions(1:numClients, :) = nextPositions(1:numClients, :);
    end
end

% Wait for final ACK: all UAVs have arrived at the last guidance target
if ~coder.target('MATLAB')
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.ACK));
end

% Exit guidance mode on all clients (second toggle)
if ~coder.target('MATLAB')
    coder.ceval('sendGuidanceToggle', int32(numClients));
    % Wait for ACK from all clients: confirms each client has exited guidance
    % mode and is back in sequential (ACK/READY) mode.
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.ACK));
end
% --------------------------------------------------------------------------

% Brief pause before closing experiment (RTL + LAND)
if coder.target('MATLAB')
    pause(2);
else
    coder.ceval('sleepMs', int32(2000));
end

% Send RTL command to all clients
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Sending RTL to client ', num2str(i)]);
    else
        coder.ceval('sendMessageType', int32(i), int32(MESSAGE_TYPE.RTL));
    end
end

% Wait for ACK from all clients
if coder.target('MATLAB')
    disp('Waiting for ACK from all clients...');
else
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.ACK));
end

% Wait for READY from all clients (returned to home)
if coder.target('MATLAB')
    disp('All UAVs returned to home.');
else
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.READY));
end

% Send LAND command to all clients
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Sending LAND to client ', num2str(i)]);
    else
        coder.ceval('sendMessageType', int32(i), int32(MESSAGE_TYPE.LAND));
    end
end

% Wait for ACK from all clients
if coder.target('MATLAB')
    disp('Waiting for ACK from all clients...');
else
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.ACK));
end

% Wait for READY from all clients (landed and disarmed)
if coder.target('MATLAB')
    disp('All UAVs landed and disarmed.');
else
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.READY));
end

% Send READY to all clients to signal mission complete
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Sending READY to client ', num2str(i)]);
    else
        coder.ceval('sendMessageType', int32(i), int32(MESSAGE_TYPE.READY));
    end
end

% Close server
if ~coder.target('MATLAB')
    coder.ceval('closeServer');
end

disp('Experiment complete.');

end
