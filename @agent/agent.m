classdef agent
    properties (SetAccess = public, GetAccess = public)
        % Identifiers
        label = "";

        % Sensor
        sensorModel;
        sensingLength = 0.05; % length parameter used by sensing function

        % Guidance
        guidanceModel;

        % State
        lastPos = NaN(1, 3); % position from previous timestep
        pos = NaN(1, 3); % current position
        vel = NaN(1, 3); % current velocity
        pan = NaN; % pan angle
        tilt = NaN; % tilt angle

        % Collision
        collisionGeometry;
        barrierFunction;
        dBarrierFunction;

        % FOV cone
        fovGeometry;

        % Communication
        comRange = NaN;
        commsGeometry = spherical;

        performance = 0;

        % Plotting
        scatterPoints;
        debug = false;
        debugFig;
        plotCommsGeometry = true;
    end

    methods (Access = public)
        [obj] = initialize(obj, pos, vel, pan, tilt, collisionGeometry, sensorModel, guidanceModel, comRange, index, label);
        [obj] = run(obj, domain, partitioning, t, index);
        [obj, f] = plot(obj, ind, f);
        updatePlots(obj);
    end
end