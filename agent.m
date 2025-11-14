classdef agent
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
        function obj = initialize(obj, pos, vel, pan, tilt, collisionGeometry, sensorModel, guidanceModel, comRange, index, label)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'agent')};
                pos (1, 3) double;
                vel (1, 3) double;
                pan (1, 1) double;
                tilt (1, 1) double;
                collisionGeometry (1, 1) {mustBeGeometry};
                sensorModel (1, 1) {mustBeSensor}
                guidanceModel (1, 1) {mustBeA(guidanceModel, 'function_handle')};
                comRange (1, 1) double = NaN;
                index (1, 1) double = NaN;
                label (1, 1) string = "";
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'agent')};
            end

            obj.pos = pos;
            obj.vel = vel;
            obj.pan = pan;
            obj.tilt = tilt;
            obj.collisionGeometry = collisionGeometry;
            obj.sensorModel = sensorModel;
            obj.guidanceModel = guidanceModel;
            obj.comRange = comRange;
            obj.index = index;
            obj.label = label;

            % Initialize FOV cone
            obj.fovGeometry = cone;
            obj.fovGeometry = obj.fovGeometry.initialize([obj.pos(1:2), 0], tan(obj.sensorModel.alphaTilt) * obj.pos(3), obj.pos(3), REGION_TYPE.FOV, sprintf("%s FOV", obj.label));
        end
        function obj = run(obj, sensingObjective, domain, partitioning)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'agent')};
                sensingObjective (1, 1) {mustBeA(sensingObjective, 'sensingObjective')};
                domain (1, 1) {mustBeGeometry};
                partitioning (:, :) double;
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'agent')};
            end

            % Do sensing
            [sensedValues, sensedPositions] = obj.sensorModel.sense(obj, sensingObjective, domain, partitioning);

            % Determine next planned position
            nextPos = obj.guidanceModel(sensedValues, sensedPositions, obj.pos);

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

            % Update FOV geometry surfaces
            for jj = 1:size(obj.fovGeometry.surface, 2)
                % Update each plot
                obj.fovGeometry.surface(jj).XData = obj.fovGeometry.surface(jj).XData + deltaPos(1);
                obj.fovGeometry.surface(jj).YData = obj.fovGeometry.surface(jj).YData + deltaPos(2);
                obj.fovGeometry.surface(jj).ZData = obj.fovGeometry.surface(jj).ZData + deltaPos(3);
            end
        end
        function [obj, f] = plot(obj, ind, f)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'agent')};
                ind (1, :) double = NaN;
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'agent')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
            end

            % Create axes if they don't already exist
            f = firstPlotSetup(f);

            % Plot points representing the agent position
            hold(f.Children(1).Children(end), "on");
            o = scatter3(f.Children(1).Children(end), obj.pos(1), obj.pos(2), obj.pos(3), 'filled', 'ko', 'SizeData', 25);
            hold(f.Children(1).Children(end), "off");

            % Check if this is a tiled layout figure
            if strcmp(f.Children(1).Type, 'tiledlayout')
                % Add to other perspectives
                o = [o; copyobj(o(1), f.Children(1).Children(2))];
                o = [o; copyobj(o(1), f.Children(1).Children(3))];
                o = [o; copyobj(o(1), f.Children(1).Children(4))];
            end

            obj.scatterPoints = o;

            % Plot collision geometry
            [obj.collisionGeometry, f] = obj.collisionGeometry.plotWireframe(ind, f);

            % Plot FOV geometry
            [obj.fovGeometry, f] = obj.fovGeometry.plot(ind, f);
        end
    end
end