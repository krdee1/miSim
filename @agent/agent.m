classdef agent
    properties (SetAccess = public, GetAccess = public)
        % Identifiers
        label = "";

        % Sensor
        sensorModel;

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
        lesserNeighbors = [];

        performance = 0;

        % Plotting
        scatterPoints;
        plotCommsGeometry = true;
    end

    properties (SetAccess = private, GetAccess = public)
        initialStepSize = 0.2;
        stepDecayRate = NaN;
    end

    methods (Access = public)
        [obj] = initialize(obj, pos, vel, pan, tilt, collisionGeometry, sensorModel, guidanceModel, comRange, index, label);
        [obj] = run(obj, domain, partitioning, t, index, agents);
        [partitioning] = partition(obj, agents, objective)
        [obj, f] = plot(obj, ind, f);
        updatePlots(obj);
    end
end