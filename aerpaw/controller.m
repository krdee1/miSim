function controller(numClients)
arguments (Input)
    numClients (1, 1) int32;
end

coder.extrinsic('disp', 'readScenarioCsv');

% Maximum clients supported (one initial position per UAV)
MAX_CLIENTS = 4;
% Three waypoints per UAV: one axis-aligned move per dimension (taxicab flyout/flyback)
MAX_TARGETS = MAX_CLIENTS * 3;

% Taxicab flyout/flyback only supports exactly 2 UAVs
if numClients ~= int32(2)
    error('Taxicab flyout/flyback requires exactly 2 UAVs');
end

% Allocate targets array (MAX_TARGETS x 3)
targets = zeros(MAX_TARGETS, 3);
numWaypoints = int32(0);
totalLoaded  = int32(0);  % pre-declare type for coder.ceval %#ok<NASGU>

% Experiment start positions from scenario CSV (N x 3)
scenarioPositions = zeros(MAX_CLIENTS, 3);

% Load experiment start positions from scenario CSV
if coder.target('MATLAB')
    disp('Loading initial positions from scenario.csv (simulation)...');
    tmpSim = miSim;
    sc = tmpSim.readScenarioCsv('aerpaw/config/scenario.csv');
    flatPos = double(sc.initialPositions);  % 1×(3*N) flat vector
    posMatrix = reshape(flatPos, 3, [])';   % N×3
    totalLoaded = int32(size(posMatrix, 1));
    scenarioPositions(1:totalLoaded, :) = posMatrix;
    % MATLAB path: send directly to scenario positions in one waypoint
    targets(1:totalLoaded, :) = posMatrix;
    numWaypoints = int32(1);
    disp(['Loaded ', num2str(double(totalLoaded)), ' initial positions']);
else
    coder.cinclude('controller_impl.h');
    filename = ['config/scenario.csv', char(0)];
    % Load into targets temporarily; copy to scenarioPositions below
    totalLoaded = coder.ceval('loadInitialPositions', coder.ref(filename), ...
                coder.ref(targets), int32(MAX_TARGETS));
    scenarioPositions(1:totalLoaded, :) = targets(1:totalLoaded, :);
    numWaypoints = int32(3);
end

% Load guidance scenario from CSV (parameters for guidance_step)
NUM_SCENARIO_PARAMS = 55;
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

% Query takeoff-pad GPS positions before sending any waypoints.
% These are also the flyback targets so each UAV lands where it took off.
initialPositions = zeros(MAX_CLIENTS, 3);
if ~coder.target('MATLAB')
    coder.ceval('sendRequestPositions', int32(numClients));
    coder.ceval('recvPositions', int32(numClients), coder.ref(initialPositions), int32(MAX_CLIENTS));
else
    % Simulation: treat scenario positions as the takeoff locations
    initialPositions(1:totalLoaded, :) = scenarioPositions(1:totalLoaded, :);
end

% ---- Build taxicab flyout waypoints ------------------------------------------
% Determine which UAV has the higher final altitude; it moves Z first so it
% clears vertical separation before the lower UAV converges on the same X/Y.
%   Higher UAV order: Z → Y → X
%   Lower  UAV order: X → Y → Z
if ~coder.target('MATLAB')
    if scenarioPositions(1, 3) >= scenarioPositions(2, 3)
        higherIdx = int32(1);
        lowerIdx  = int32(2);
    else
        higherIdx = int32(2);
        lowerIdx  = int32(1);
    end

    hBase = double(higherIdx - 1) * double(numWaypoints);
    lBase = double(lowerIdx  - 1) * double(numWaypoints);

    % Higher UAV: Z first
    targets(hBase + 1, :) = [initialPositions(higherIdx,1), initialPositions(higherIdx,2), scenarioPositions(higherIdx,3)];
    targets(hBase + 2, :) = [initialPositions(higherIdx,1), scenarioPositions(higherIdx,2), scenarioPositions(higherIdx,3)];
    targets(hBase + 3, :) =  scenarioPositions(higherIdx, :);

    % Lower UAV: X first
    targets(lBase + 1, :) = [scenarioPositions(lowerIdx,1), initialPositions(lowerIdx,2), initialPositions(lowerIdx,3)];
    targets(lBase + 2, :) = [scenarioPositions(lowerIdx,1), scenarioPositions(lowerIdx,2), initialPositions(lowerIdx,3)];
    targets(lBase + 3, :) =  scenarioPositions(lowerIdx, :);
end
% ------------------------------------------------------------------------------

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
% Number of guidance steps comes from maxIter in scenario.csv (scenarioParams(2))
% so the controller and the agent step-decay schedule stay in sync.
if coder.target('MATLAB')
    MAX_GUIDANCE_STEPS = int32(sc.maxIter);
else
    MAX_GUIDANCE_STEPS = int32(scenarioParams(2));
end

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
    % Simulation: seed positions from scenario positions so agents don't start at origin
    positions(1:totalLoaded, :) = scenarioPositions(1:totalLoaded, :);
end
guidance_step(positions(1:numClients, :), true, ...
              scenarioParams, obstacleMin, obstacleMax, numObstacles);

% Main guidance loop (event-triggered)
for step = 1:MAX_GUIDANCE_STEPS
    if ~coder.target('MATLAB')
        coder.ceval('setGuidanceStep', int32(step), int32(MAX_GUIDANCE_STEPS));
    end

    % Run one guidance step: feed current GPS positions in, get targets out
    nextPositions = guidance_step(positions(1:numClients, :), false, ...
                                  scenarioParams, obstacleMin, obstacleMax, numObstacles);

    % Send target to each client
    for i = 1:numClients
        target = nextPositions(i, :);
        if ~coder.target('MATLAB')
            coder.ceval('sendTarget', int32(i), coder.ref(target));
        else
            disp(['[step ', num2str(step), '] target UAV ', num2str(i), ': ', num2str(target)]);
        end
    end

    % Wait for ACK from all clients (each UAV ACKs when it arrives at its target)
    if ~coder.target('MATLAB')
        coder.ceval('waitForAllMessageType', int32(numClients), ...
                    int32(MESSAGE_TYPE.ACK));
    else
        disp(['[guidance] step ', num2str(step), ': all UAVs arrived']);
    end

    % Request current GPS positions from all clients
    if ~coder.target('MATLAB')
        coder.ceval('sendRequestPositions', int32(numClients));
        coder.ceval('recvPositions', int32(numClients), coder.ref(positions), int32(MAX_CLIENTS));
    else
        % Simulation: advance positions to guidance outputs for closed-loop feedback
        positions(1:numClients, :) = nextPositions(1:numClients, :);
    end
end

% Exit guidance mode on all clients (second toggle)
if ~coder.target('MATLAB')
    coder.ceval('sendGuidanceToggle', int32(numClients));
    % Wait for ACK from all clients: confirms each client has finished its
    % last guidance navigation and is back in sequential (ACK/READY) mode.
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.ACK));
    % Reset step counter so post-guidance logging carries no step prefix.
    coder.ceval('setGuidanceStep', int32(0), int32(MAX_GUIDANCE_STEPS));
end
% --------------------------------------------------------------------------

% ---- Taxicab flyback: return each UAV to its takeoff-pad position ---------
% The UAV that ended guidance at the higher altitude moves Z last (X → Y → Z)
% so it stays high while the lower UAV descends first, maintaining separation
% as both converge back on their respective home X/Y positions.
NUM_RETURN_WP = int32(3);
returnTargets = zeros(MAX_TARGETS, 3);

if ~coder.target('MATLAB')
    if positions(1, 3) >= positions(2, 3)
        higherRetIdx = int32(1);
        lowerRetIdx  = int32(2);
    else
        higherRetIdx = int32(2);
        lowerRetIdx  = int32(1);
    end

    hRetBase = double(higherRetIdx - 1) * double(NUM_RETURN_WP);
    lRetBase = double(lowerRetIdx  - 1) * double(NUM_RETURN_WP);

    % Higher post-guidance UAV: X → Y → Z (descend last)
    returnTargets(hRetBase + 1, :) = [initialPositions(higherRetIdx,1), positions(higherRetIdx,2),        positions(higherRetIdx,3)];
    returnTargets(hRetBase + 2, :) = [initialPositions(higherRetIdx,1), initialPositions(higherRetIdx,2), positions(higherRetIdx,3)];
    returnTargets(hRetBase + 3, :) =  initialPositions(higherRetIdx, :);

    % Lower post-guidance UAV: Z → Y → X (descend first)
    returnTargets(lRetBase + 1, :) = [positions(lowerRetIdx,1),        positions(lowerRetIdx,2),        initialPositions(lowerRetIdx,3)];
    returnTargets(lRetBase + 2, :) = [positions(lowerRetIdx,1),        initialPositions(lowerRetIdx,2), initialPositions(lowerRetIdx,3)];
    returnTargets(lRetBase + 3, :) =  initialPositions(lowerRetIdx, :);

    for w = 1:NUM_RETURN_WP
        for i = 1:numClients
            retIdx = double(i - 1) * double(NUM_RETURN_WP) + w;
            retTarget = returnTargets(retIdx, :);
            coder.ceval('sendTarget', int32(i), coder.ref(retTarget));
        end
        coder.ceval('waitForAllMessageType', int32(numClients), int32(MESSAGE_TYPE.ACK));
        coder.ceval('waitForAllMessageType', int32(numClients), int32(MESSAGE_TYPE.READY));
    end
else
    disp('Taxicab return (simulation): UAVs commanded back to takeoff positions.');
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
