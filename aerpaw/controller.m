function controller(numClients)
arguments (Input)
    numClients (1, 1) int32;
end

coder.extrinsic('disp', 'loadTargetsFromYaml');

% Maximum clients supported
MAX_CLIENTS = 4;

% Allocate targets array (MAX_CLIENTS x 3)
targets = zeros(MAX_CLIENTS, 3);

% Load targets from YAML config file
if coder.target('MATLAB')
    disp('Loading targets from config.yaml (simulation)...');
    targetsLoaded = loadTargetsFromYaml('aerpaw/config/config.yaml');
    numTargets = min(size(targetsLoaded, 1), numClients);
    targets(1:numTargets, :) = targetsLoaded(1:numTargets, :);
    disp(['Loaded ', num2str(numTargets), ' targets']);
else
    coder.cinclude('controller_impl.h');
    % Define filename as null-terminated character array for C compatibility
    filename = ['config/config.yaml', char(0)];
    % loadTargets fills targets array (column-major for MATLAB compatibility)
    coder.ceval('loadTargets', coder.ref(filename), ...
                coder.ref(targets), int32(MAX_CLIENTS));
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

% Send target coordinates to each client
for i = 1:numClients
    % Get target for this client (1x3 array)
    target = targets(i, :);

    if coder.target('MATLAB')
        disp(['Sending ', char(MESSAGE_TYPE.TARGET), ' to client ', num2str(i), ': ', ...
              num2str(target(1)), ',', num2str(target(2)), ',', num2str(target(3))]);
    else
        coder.ceval('sendTarget', int32(i), coder.ref(target));
    end
end

% Wait for ACK from all clients
if coder.target('MATLAB')
    disp(['Waiting for ', char(MESSAGE_TYPE.ACK), ' from all clients...']);
    disp('All acknowledgments received.');
else
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.ACK));
end

% Wait for READY signals from all clients
if coder.target('MATLAB')
    disp(['Waiting for ', char(MESSAGE_TYPE.READY), ' from all clients...']);
    disp('All UAVs at target positions.');
else
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.READY));
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
        disp(['Sending ', char(MESSAGE_TYPE.RTL), ' to client ', num2str(i)]);
    else
        coder.ceval('sendMessageType', int32(i), int32(MESSAGE_TYPE.RTL));
    end
end

% Wait for ACK from all clients
if coder.target('MATLAB')
    disp(['Waiting for ', char(MESSAGE_TYPE.ACK), ' from all clients...']);
else
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.ACK));
end

% Wait for READY from all clients (returned to home)
if coder.target('MATLAB')
    disp(['Waiting for ', char(MESSAGE_TYPE.READY), ' from all clients...']);
    disp('All UAVs returned to home.');
else
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.READY));
end

% Send LAND command to all clients
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Sending ', char(MESSAGE_TYPE.LAND), ' to client ', num2str(i)]);
    else
        coder.ceval('sendMessageType', int32(i), int32(MESSAGE_TYPE.LAND));
    end
end

% Wait for ACK from all clients
if coder.target('MATLAB')
    disp(['Waiting for ', char(MESSAGE_TYPE.ACK), ' from all clients...']);
else
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.ACK));
end

% Wait for READY from all clients (landed and disarmed)
if coder.target('MATLAB')
    disp(['Waiting for ', char(MESSAGE_TYPE.READY), ' from all clients...']);
    disp('All UAVs landed and disarmed.');
else
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.READY));
end

% Send READY to all clients to signal mission complete
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Sending ', char(MESSAGE_TYPE.READY), ' to client ', num2str(i)]);
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
