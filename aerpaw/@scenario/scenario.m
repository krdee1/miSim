classdef scenario
    properties (Access = public)
        % Experiment
        domain = rectangularPrism;
        inits;

        % State
        opMode = OPMODE.INVALID;

        % UAVs
        uavs;

        % Communications
        timeout = 2; % seconds
        basePort = 3330;
        portOffset = 1110;
        ports;
        udp;

    end

    methods(Access = public)
        [obj] = initialize(obj, initsPath);
        [obj] = run(obj);
        [obj] = setup(obj);
        u = sendTarget(u, pos);
    end
end