function obj = initialize(obj, port, timeout, label)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "uav")};
        port (1, 1) double;
        timeout (1, 1) double;
        label (1, 1) string;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "uav")};
    end
    obj.label = label;
    obj.port = port;
    obj.timeout = timeout;

    % Branch here depending on environment
    if coder.target("MATLAB")
        obj.pos = [0, 0, 20] + rand(1, 3) * 10; % just put it somewhere so we can plot it
        
    elseif coder.target("C++")
        % initialize ip/port
        coder.cinclude('udp_comm.h');
        coder.ceval('initComs', "ip", obj.port);
    end

    obj.opMode = OPMODE.INITIALIZED;

end