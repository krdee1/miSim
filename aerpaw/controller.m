function controller(numClients)
arguments (Input)
    numClients (1, 1) int32;
end

coder.extrinsic('disp', 'readmatrix');

% Maximum clients supported
MAX_CLIENTS = 4;

% Allocate targets array (MAX_CLIENTS x 3)
targets = zeros(MAX_CLIENTS, 3);

% Load targets from file
if coder.target('MATLAB')
    disp('Loading targets from file (simulation)...');
    targetsLoaded = readmatrix('aerpaw/config/targets.txt');
    numTargets = min(size(targetsLoaded, 1), numClients);
    targets(1:numTargets, :) = targetsLoaded(1:numTargets, :);
    disp(['Loaded ', num2str(numTargets), ' targets']);
else
    coder.cinclude('controller_impl.h');
    % Define filename as null-terminated character array for C compatibility
    filename = ['config/targets.txt', char(0)];
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
        disp(['Sending TARGET to client ', num2str(i), ': ', ...
              num2str(target(1)), ',', num2str(target(2)), ',', num2str(target(3))]);
    else
        coder.ceval('sendTarget', int32(i), coder.ref(target));
    end
end

% Receive TARGET acknowledgments
targetAcks = zeros(1, numClients, 'int32');
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Waiting for ACK:TARGET from client ', num2str(i)]);
        targetAcks(i) = 1;  % Simulate successful ACK
    else
        targetAcks(i) = coder.ceval('receiveTargetAck', int32(i));
    end
end

% Check all ACKs received
if coder.target('MATLAB')
    disp(['Target ACKs received: ', num2str(targetAcks)]);
end

% Wait for READY signals (UAVs have reached their targets)
readySignals = zeros(1, numClients, 'int32');
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Waiting for READY from client ', num2str(i)]);
        readySignals(i) = 1;  % Simulate READY
    else
        readySignals(i) = coder.ceval('waitForReady', int32(i));
    end
end

% Check all READY signals received
if coder.target('MATLAB')
    disp(['Ready signals received: ', num2str(readySignals)]);
    disp('All UAVs at target positions.');
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
        coder.ceval('sendRTL', int32(i));
    end
end

% Wait for RTL_COMPLETE from all clients (simultaneously using select())
if coder.target('MATLAB')
    disp('Waiting for RTL_COMPLETE from all clients...');
    disp('All UAVs returned to home.');
else
    coder.ceval('waitForAllRTLComplete', int32(numClients));
end

% Send LAND command to all clients
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Sending LAND to client ', num2str(i)]);
    else
        coder.ceval('sendLAND', int32(i));
    end
end

% Wait for LAND_COMPLETE from all clients (simultaneously using select())
if coder.target('MATLAB')
    disp('Waiting for LAND_COMPLETE from all clients...');
    disp('All UAVs landed and disarmed.');
else
    coder.ceval('waitForAllLANDComplete', int32(numClients));
end

% Send FINISHED to all clients before closing
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Sending FINISHED to client ', num2str(i)]);
    else
        coder.ceval('sendFinished', int32(i));
    end
end

% Close server
if ~coder.target('MATLAB')
    coder.ceval('closeServer');
end

disp('Experiment complete.');

end