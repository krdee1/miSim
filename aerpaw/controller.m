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
    tmpSim = miSim;
    sc = tmpSim.readScenarioCsv('aerpaw/config/scenario.csv');
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
NUM_SCENARIO_PARAMS = 27;
MAX_OBSTACLES_CTRL  = int32(8);
scenarioParams = zeros(1, NUM_SCENARIO_PARAMS);
obstacleMin    = zeros(MAX_OBSTACLES_CTRL, 3);
obstacleMax    = zeros(MAX_OBSTACLES_CTRL, 3);
numObstacles   = int32(0);
if ~coder.target('MATLAB')
    coder.cinclude('controller_impl.h');
    scenarioFilename = ['config/scenario.csv', char(0)];
    coder.ceval('loadScenario', coder.ref(scenarioFilename), coder.ref(scenarioParams));
    numObstacles = coder.ceval('loadObstacles', coder.ref(scenarioFilename), ...
                         coder.ref(obstacleMin), coder.ref(obstacleMax), ...
                         int32(MAX_OBSTACLES_CTRL));
end
% On MATLAB path, scenarioParams and obstacle arrays are left as zeros.
% guidance_step's MATLAB path loads parameters directly from scenario.csv
% via sim.initializeFromCsv and does not use these arrays.

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
% Guidance parameters (adjust here and recompile as needed)
MAX_GUIDANCE_STEPS = int32(100); % number of guidance iterations
GUIDANCE_RATE_MS   = int32(5000); % ms between iterations (0.2 Hz default)

% Enter guidance mode on all clients
if ~coder.target('MATLAB')
    coder.ceval('sendGuidanceToggle', int32(numClients));
end

% Request initial GPS positions and initialise guidance algorithm
positions = zeros(MAX_CLIENTS, 3);
if ~coder.target('MATLAB')
    coder.ceval('sendRequestPositions', int32(numClients));
    coder.ceval('recvPositions', int32(numClients), coder.ref(positions), int32(MAX_CLIENTS));
else
    % Simulation: seed positions from CSV waypoints so agents don't start at origin
    positions(1:totalLoaded, :) = targets(1:totalLoaded, :);
end
guidance_step(positions(1:numClients, :), true, ...
              scenarioParams, obstacleMin, obstacleMax, numObstacles);

% Main guidance loop
for step = 1:MAX_GUIDANCE_STEPS
    % Query current GPS positions from all clients
    if ~coder.target('MATLAB')
        coder.ceval('sendRequestPositions', int32(numClients));
        coder.ceval('recvPositions', int32(numClients), coder.ref(positions), int32(MAX_CLIENTS));
    end

    % Run one guidance step: feed GPS positions in, get targets out
    nextPositions = guidance_step(positions(1:numClients, :), false, ...
                                  scenarioParams, obstacleMin, obstacleMax, numObstacles);

    % Send target to each client (no ACK/READY expected in guidance mode)
    for i = 1:numClients
        target = nextPositions(i, :);
        if ~coder.target('MATLAB')
            coder.ceval('sendTarget', int32(i), coder.ref(target));
        else
            disp(['[guidance] target UAV ', num2str(i), ': ', num2str(target)]);
        end
    end

    % Simulation: advance positions to guidance outputs for closed-loop feedback
    if coder.target('MATLAB')
        positions(1:numClients, :) = nextPositions(1:numClients, :);
    end

    % Wait for the guidance rate interval before the next iteration
    if ~coder.target('MATLAB')
        coder.ceval('sleepMs', int32(GUIDANCE_RATE_MS));
    end
end

% Exit guidance mode on all clients (second toggle)
if ~coder.target('MATLAB')
    coder.ceval('sendGuidanceToggle', int32(numClients));
    % Wait for ACK from all clients: confirms each client has finished its
    % last guidance navigation and is back in sequential (ACK/READY) mode.
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.ACK));
end
% --------------------------------------------------------------------------

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
