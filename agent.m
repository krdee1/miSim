classdef agent
    properties (SetAccess = private, GetAccess = public)
        % Identifiers
        index = NaN;
        label = "";

        % Sensor
        sensingFunction = @(r) 0.5; % probability of detection as a function of range
        sensingLength = 0.05; % length parameter used by sensing function

        % State
        pos = NaN(1, 3);
        vel = NaN(1, 3);
        cBfromC = NaN(3); % DCM body from sim cartesian (assume fixed for now)

        % Collision
        collisionGeometry;

        % Communication
        comRange = NaN;
    end

    methods (Access = public)
        function obj = initialize(obj, pos, vel, cBfromC, collisionGeometry, sensingFunction, sensingLength, comRange, index, label)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'agent')};
                pos (1, 3) double;
                vel (1, 3) double;
                cBfromC (3, 3) double {mustBeDcm};
                collisionGeometry (1, 1) {mustBeGeometry};
                sensingFunction (1, 1) {mustBeA(sensingFunction, 'function_handle')} = @(r) 0.5;
                sensingLength (1, 1) double = NaN;
                comRange (1, 1) double = NaN;
                index (1, 1) double = NaN;
                label (1, 1) string = "";
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'agent')};
            end

            obj.pos = pos;
            obj.vel = vel;
            obj.cBfromC = cBfromC;
            obj.collisionGeometry = collisionGeometry;
            obj.sensingFunction = sensingFunction;
            obj.sensingLength = sensingLength;
            obj.comRange = comRange;
            obj.index = index;
            obj.label = label;
        end
        function obj = run(obj, objectiveFunction)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'agent')};
                objectiveFunction (1, 1) {mustBeA(objectiveFunction, 'function_handle')};
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'agent')};
            end

            % Do sensing to determine target position
            nextPos = obj.sensingFunction(objectiveFunction, obj.pos, obj.sensingLength);

            % Move to next position
            % (dynamics not modeled at this time)
            obj.pos = nextPos;

        end
        function f = plot(obj, f)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'agent')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
            end

            % Create axes if they don't already exist
            f = firstPlotSetup(f);

            % Plot points representing the agent position
            hold(f.CurrentAxes, "on");
            o = scatter3(obj.pos(1), obj.pos(2), obj.pos(3), 'filled', 'ko', 'SizeData', 25);
            hold(f.CurrentAxes, "off");

            % Check if this is a tiled layout figure
            if strcmp(f.Children(1).Type, 'tiledlayout')
                % Add to other perspectives
                copyobj(o, f.Children(1).Children(2));
                copyobj(o, f.Children(1).Children(3));
                copyobj(o, f.Children(1).Children(5));
            end
        end
    end
end