function controller(numClients)
arguments (Input)
    numClients (1, 1) int32;
end

coder.extrinsic('disp', 'loadTargetsFromYaml');

% Maximum clients and waypoints supported
MAX_CLIENTS = 4;
MAX_WAYPOINTS = 10;
MAX_TARGETS = MAX_CLIENTS * MAX_WAYPOINTS;

% Allocate targets array (MAX_TARGETS x 3)
targets = zeros(MAX_TARGETS, 3);
numWaypoints = int32(0);

% Load targets from YAML config file
if coder.target('MATLAB')
    disp('Loading targets from server.yaml (simulation)...');
    targetsLoaded = loadTargetsFromYaml('aerpaw/config/server.yaml');
    totalLoaded = size(targetsLoaded, 1);
    targets(1:totalLoaded, :) = targetsLoaded(1:totalLoaded, :);
    numWaypoints = int32(totalLoaded / numClients);
    disp(['Loaded ', num2str(numWaypoints), ' waypoints per client']);
else
    coder.cinclude('controller_impl.h');
    % Define filename as null-terminated character array for C compatibility
    filename = ['config/server.yaml', char(0)];
    % loadTargets fills targets array (column-major for MATLAB compatibility)
    totalLoaded = int32(0);
    totalLoaded = coder.ceval('loadTargets', coder.ref(filename), ...
                coder.ref(targets), int32(MAX_TARGETS));
    numWaypoints = totalLoaded / int32(numClients);
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

% Wait for user input before closing experiment
if coder.target('MATLAB')
    input('Press Enter to close experiment (RTL + LAND): ', 's');
else
    coder.ceval('waitForUserInput');
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
