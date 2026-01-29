function obj = initialize(obj, initsPath)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'scenario')};
        initsPath (1, 1) string;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'scenario')};
    end

    obj.inits = load(initsPath);

    % Instantiate the correct number of UAVs
    obj.uavs = cell(obj.inits.numAgents, 1);
    [obj.uavs{:}] = deal(uav);

    % Configure ports to broadcast to drones
    obj.ports = repmat(obj.basePort, [obj.inits.numAgents, 1]) + (1:obj.inits.numAgents)';
    obj.udp = cell(obj.inits.numAgents, 1);
    for ii = 1:obj.inits.numAgents
        obj.udp{ii} = udpport("IPV4", "LocalPort", obj.ports(ii), "Tag", sprintf("UAV %d", ii), "Timeout", obj.timeout);
    end

    % Initialize UAVs in scenario's knowledge
    for ii = 1:obj.inits.numAgents
        obj.uavs{ii} = obj.uavs{ii}.initialize(obj.ports(ii) + obj.portOffset, obj.timeout, sprintf("UAV %d", ii));
    end

    % Update opMode
    obj.opMode = OPMODE.INITIALIZED;
end