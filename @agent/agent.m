classdef agent
    properties (SetAccess = public, GetAccess = public)
        % Identifiers
        label = "";

        % State
        lastPos = NaN(1, 3); % position from previous timestep
        pos = NaN(1, 3); % current position

        % Sensor
        sensorModel;

        % Collision
        collisionGeometry;

        % FOV cone
        fovGeometry;

        % Communication
        commsGeometry;
        lesserNeighbors = zeros(1, 0);

        % Performance
        performance = 0;

        % Plotting
        scatterPoints;
        plotCommsGeometry = true;
    end

    properties (SetAccess = private, GetAccess = public)
        initialStepSize = NaN;
        stepDecayRate = NaN;
    end

    methods (Access = public)
        function obj = agent()
            arguments (Input)
            end
            arguments (Output)
                obj (1, 1) agent
            end
            obj.collisionGeometry = spherical;
            obj.sensorModel = sigmoidSensor;
            obj.fovGeometry = cone;
            obj.commsGeometry = spherical;
        end
        [obj] = initialize(obj, pos, pan, tilt, collisionGeometry, sensorModel, guidanceModel, comRange, index, label);
        [obj] = run(obj, domain, partitioning, t, index, agents);
        [partitioning] = partition(obj, agents, objective)
        [obj, f] = plot(obj, ind, f);
        updatePlots(obj);
    end
end