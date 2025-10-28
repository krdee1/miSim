classdef agent
    properties (SetAccess = private, GetAccess = public)
        % Identifiers
        index = NaN;
        label = "";

        % Sensor
        sensingFunction = @(r) 0.5; % probability of detection as a function of range
        sensingLength = 0.05; % length parameter used by sensing function

        % State
        lastPos = NaN(1, 3); % position from previous timestep
        pos = NaN(1, 3); % current position
        vel = NaN(1, 3); % current velocity
        cBfromC = NaN(3); % current DCM body from sim cartesian (assume fixed for now)

        % Collision
        collisionGeometry;

        % Communication
        comRange = NaN;

        % Plotting
        scatterPoints;
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
        function obj = run(obj, objectiveFunction, domain)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'agent')};
                objectiveFunction (1, 1) {mustBeA(objectiveFunction, 'function_handle')};
                domain (1, 1) {mustBeGeometry};
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'agent')};
            end

            % Do sensing to determine target position
            nextPos = obj.sensingFunction(objectiveFunction, domain, obj.pos, obj.sensingLength);

            % Move to next position
            % (dynamics not modeled at this time)
            obj.lastPos = obj.pos;
            obj.pos = nextPos;

            % Calculate movement
            d = obj.pos - obj.collisionGeometry.center;

            % Reinitialize collision geometry in the new position
            obj.collisionGeometry = obj.collisionGeometry.initialize([obj.collisionGeometry.minCorner; obj.collisionGeometry.maxCorner] + d, obj.collisionGeometry.tag, obj.collisionGeometry.label);
        end
        function updatePlots(obj)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'agent')};
            end
            arguments (Output)
            end

            % Scatterplot point positions
            for ii = 1:size(obj.scatterPoints, 1)
                obj.scatterPoints(ii).XData = obj.pos(1);
                obj.scatterPoints(ii).YData = obj.pos(2);
                obj.scatterPoints(ii).ZData = obj.pos(3);
            end

            % Find change in agent position since last timestep
            deltaPos = obj.pos - obj.lastPos;

            % Collision geometry edges
            for jj = 1:size(obj.collisionGeometry.lines, 2)
                % Update plotting
                for ii = 1:size(obj.collisionGeometry.lines(:, jj), 1)
                    obj.collisionGeometry.lines(ii, jj).XData = obj.collisionGeometry.lines(ii, jj).XData + deltaPos(1);
                    obj.collisionGeometry.lines(ii, jj).YData = obj.collisionGeometry.lines(ii, jj).YData + deltaPos(2);
                    obj.collisionGeometry.lines(ii, jj).ZData = obj.collisionGeometry.lines(ii, jj).ZData + deltaPos(3);
                end
            end

            % Network connections
        end
        function [obj, f] = plot(obj, f)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'agent')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'agent')};
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
                o = [o; copyobj(o(1), f.Children(1).Children(2))];
                o = [o; copyobj(o(1), f.Children(1).Children(3))];
                o = [o; copyobj(o(1), f.Children(1).Children(5))];
            end

            obj.scatterPoints = o;

            % Plot collision geometry
            [obj.collisionGeometry, f] = obj.collisionGeometry.plotWireframe(f);
        end
    end
end