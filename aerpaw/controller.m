function controller(numClients)
arguments (Input)
    numClients (1, 1) int32;
end

coder.extrinsic('disp', 'readScenarioCsv');

% Maximum clients supported (one initial position per UAV)
MAX_CLIENTS = 4;
% Three waypoints per UAV: stage (climb) → traverse (XY) → settle (descend)
MAX_TARGETS = MAX_CLIENTS * 3;

% Altitude-staggered flyout/flyback currently supports exactly 2 UAVs
if numClients ~= int32(2)
    error('Altitude-staggered flyout/flyback requires exactly 2 UAVs');
end

% Minimum vertical separation maintained between UAVs during XY transit (metres)
TRANSIT_SEP = 30.0;

% Allocate targets array (MAX_TARGETS x 3)
targets = zeros(MAX_TARGETS, 3);
numWaypoints = int32(0); %#ok<NASGU>
totalLoaded  = int32(0);

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
NUM_SCENARIO_PARAMS = 65;
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

% ---- Build altitude-staggered flyout waypoints --------------------------------
% Each UAV follows three phases:
%   WP1: Climb to transit altitude at pad XY
%   WP2: Traverse to final XY at transit altitude  (>= TRANSIT_SEP vertical gap)
%   WP3: Descend to final altitude at final XY
%
% Transit altitude assignment: the UAV with the highest final altitude keeps
% that altitude as its transit altitude; the other UAV climbs TRANSIT_SEP above
% it, guaranteeing >= 30 m vertical separation during the simultaneous XY
% traverse (WP2).  Both transit altitudes are well above minAlt.
if ~coder.target('MATLAB')
    if scenarioPositions(1, 3) >= scenarioPositions(2, 3)
        higherIdx = int32(1);
        lowerIdx  = int32(2);
    else
        higherIdx = int32(2);
        lowerIdx  = int32(1);
    end

    transitAlt = zeros(1, MAX_CLIENTS);
    transitAlt(lowerIdx)  = scenarioPositions(lowerIdx, 3);
    transitAlt(higherIdx) = scenarioPositions(lowerIdx, 3) + TRANSIT_SEP;

    for iUAV = 1:numClients
        base = double(iUAV - 1) * double(numWaypoints);
        targets(base + 1, :) = [initialPositions(iUAV,1), initialPositions(iUAV,2), transitAlt(iUAV)];
        targets(base + 2, :) = [scenarioPositions(iUAV,1), scenarioPositions(iUAV,2), transitAlt(iUAV)];
        targets(base + 3, :) =  scenarioPositions(iUAV, :);
    end
end
% -------------------------------------------------------------------------------

% Waypoint loop
if coder.target('MATLAB')
    % Simulation: simple lockstep (collisions are not modelled here).
    for w = 1:numWaypoints
        for i = 1:numClients
            targetIdx = (i - 1) * numWaypoints + w;
            target = targets(targetIdx, :);
            disp(['Sending TARGET to client ', num2str(i), ' (waypoint ', num2str(w), '): ', ...
                  num2str(target(1)), ',', num2str(target(2)), ',', num2str(target(3))]);
        end
        disp('Waiting for ACK from all clients...');
        disp(['All UAVs arrived at waypoint ', num2str(w)]);
    end
else
    % ---- Staggered flyout -----------------------------------------------------
    % Separate the two UAVs in TIME so they are never doing the same vertical
    % climb at the same moment over their ~10 m-separated pads (which can trip
    % the proximity guard on a spurious GPS sample). The UAV that transits
    % highest leads by exactly one event.
    %
    % Events per UAV: 1 = TAKEOFF (25 m), 2 = WP1 (climb at pad), 3 = WP2
    % (traverse), 4 = WP3 (descend). At each step the higher UAV does event
    % `step` while the lower UAV does event `step-1`; both are dispatched, then
    % we collect ACK and then READY (arrival) from each active UAV.
    numEvents = double(numWaypoints) + 1;   % 3 waypoints + 1 takeoff
    for step = 1:(numEvents + 1)
        hEvent = step;
        lEvent = step - 1;
        hActive = (hEvent >= 1) && (hEvent <= numEvents);
        lActive = (lEvent >= 1) && (lEvent <= numEvents);

        % --- Dispatch this step's event to each active UAV ---
        if hActive
            if hEvent == 1
                coder.ceval('sendMessageType', higherIdx, int32(MESSAGE_TYPE.TAKEOFF));
            else
                hRow = (double(higherIdx) - 1) * double(numWaypoints) + (hEvent - 1);
                hTarget = targets(hRow, :);
                coder.ceval('sendTarget', higherIdx, coder.ref(hTarget));
            end
        end
        if lActive
            if lEvent == 1
                coder.ceval('sendMessageType', lowerIdx, int32(MESSAGE_TYPE.TAKEOFF));
            else
                lRow = (double(lowerIdx) - 1) * double(numWaypoints) + (lEvent - 1);
                lTarget = targets(lRow, :);
                coder.ceval('sendTarget', lowerIdx, coder.ref(lTarget));
            end
        end

        % --- Collect ACK (command received) from each active UAV ---
        if hActive
            coder.ceval('waitForClientMessageType', higherIdx, int32(MESSAGE_TYPE.ACK));
        end
        if lActive
            coder.ceval('waitForClientMessageType', lowerIdx, int32(MESSAGE_TYPE.ACK));
        end

        % --- Collect READY (arrived at event) from each active UAV ---
        if hActive
            coder.ceval('waitForClientMessageType', higherIdx, int32(MESSAGE_TYPE.READY));
        end
        if lActive
            coder.ceval('waitForClientMessageType', lowerIdx, int32(MESSAGE_TYPE.READY));
        end
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

% ---- Altitude-staggered flyback -------------------------------------------
% Mirror of flyout: stage (climb) → traverse (XY) → settle (descend to pad).
% Transit altitudes are based on post-guidance positions so the UAV that ended
% higher stays highest, guaranteeing >= TRANSIT_SEP vertical clearance during
% the simultaneous XY traverse.
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

    returnTransitAlt = zeros(1, MAX_CLIENTS);
    returnTransitAlt(lowerRetIdx)  = positions(lowerRetIdx, 3);
    returnTransitAlt(higherRetIdx) = positions(lowerRetIdx, 3) + TRANSIT_SEP;

    for iUAV = 1:numClients
        retBase = double(iUAV - 1) * double(NUM_RETURN_WP);
        returnTargets(retBase + 1, :) = [positions(iUAV,1),         positions(iUAV,2),         returnTransitAlt(iUAV)];
        returnTargets(retBase + 2, :) = [initialPositions(iUAV,1),  initialPositions(iUAV,2),  returnTransitAlt(iUAV)];
        % Final return waypoint: pad XY from the pre-takeoff GPS query, but a fixed
        % 25 m up. That query ran while the UAV was on the ground, so its ENU
        % up-component is ~0 (often slightly negative) — using it directly commands
        % a negative altitude. RTL/LAND perform the actual touchdown from here.
        returnTargets(retBase + 3, :) = [initialPositions(iUAV,1), initialPositions(iUAV,2), 25.0];
    end

    % Staggered flyback: same higher-leads-by-one pattern as the flyout, but
    % over the 3 return waypoints only (the UAVs are already airborne, so there
    % is no TAKEOFF event). The higher UAV climbs/traverses/descends one event
    % ahead of the lower, so they never share the same descent over their pads.
    numReturnEvents = double(NUM_RETURN_WP);   % 3
    for step = 1:(numReturnEvents + 1)
        hEvent = step;
        lEvent = step - 1;
        hActive = (hEvent >= 1) && (hEvent <= numReturnEvents);
        lActive = (lEvent >= 1) && (lEvent <= numReturnEvents);

        if hActive
            hRow = (double(higherRetIdx) - 1) * double(NUM_RETURN_WP) + hEvent;
            hTarget = returnTargets(hRow, :);
            coder.ceval('sendTarget', higherRetIdx, coder.ref(hTarget));
        end
        if lActive
            lRow = (double(lowerRetIdx) - 1) * double(NUM_RETURN_WP) + lEvent;
            lTarget = returnTargets(lRow, :);
            coder.ceval('sendTarget', lowerRetIdx, coder.ref(lTarget));
        end

        if hActive
            coder.ceval('waitForClientMessageType', higherRetIdx, int32(MESSAGE_TYPE.ACK));
        end
        if lActive
            coder.ceval('waitForClientMessageType', lowerRetIdx, int32(MESSAGE_TYPE.ACK));
        end

        if hActive
            coder.ceval('waitForClientMessageType', higherRetIdx, int32(MESSAGE_TYPE.READY));
        end
        if lActive
            coder.ceval('waitForClientMessageType', lowerRetIdx, int32(MESSAGE_TYPE.READY));
        end
    end
else
    disp('Altitude-staggered return (simulation): UAVs commanded back to takeoff positions.');
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
