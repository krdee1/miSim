classdef agent
    properties (Access = public)
        performance = NaN; % current individual sensor performance on partition
    end
    properties (SetAccess = private, GetAccess = public)
        % Identifiers
        index = NaN;
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

        % FOV cone
        fovGeometry;

        % Communication
        comRange = NaN;

        % Plotting
        scatterPoints;
    end

    methods (Access = public)
        [obj] = initialize(obj, pos, vel, pan, tilt, collisionGeometry, sensorModel, guidanceModel, comRange, index, label);
        [obj] = run(obj, sensingObjective, domain, partitioning);
        [obj, f] = plot(obj, ind, f);
        updatePlots(obj);
    end
end