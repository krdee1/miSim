function controller(numClients)
arguments (Input)
    numClients (1, 1) int32;
end

coder.extrinsic('disp');

% Initialize server
if coder.target('MATLAB')
    disp('Initializing server (simulation)...');
else
    coder.cinclude('controller_impl.h');
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


% Send messages to clients
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Sending message to client ', num2str(i)]);
    else
        coder.ceval('sendMessage', int32(i));
    end
end

% Receive acknowledgements
acksReceived = zeros(1, numClients, 'int32');
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Receiving ACK from client ', num2str(i)]);
        acksReceived(i) = 1;  % Simulate successful ACK
    else
        acksReceived(i) = coder.ceval('receiveAck', int32(i));
    end
end


% Digest ACKs
if coder.target('MATLAB')
    disp(['All ACKs received: ', num2str(acksReceived)]);
end


% Close server
if ~coder.target('MATLAB')
    coder.ceval('closeServer');
end

end