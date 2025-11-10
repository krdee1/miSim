classdef miSim
    % multiagent interconnection simulation

    % Simulation parameters
    properties (SetAccess = private, GetAccess = public)
        timestep = NaN; % delta time interval for simulation iterations
        maxIter = NaN; % maximum number of simulation iterations
        domain = rectangularPrism;
        objective = sensingObjective;
        obstacles = cell(0, 1); % geometries that define obstacles within the domain
        agents = cell(0, 1); % agents that move within the domain
        adjacency = NaN; % Adjacency matrix representing communications network graph
    end

    properties (Access = private)
        v = VideoWriter(fullfile('sandbox', strcat(string(datetime('now'), 'yyyy_MM_dd_HH_mm_ss'), '_miSimHist')));
        connectionsPlot; % objects for lines connecting agents in spatial plots
        graphPlot; % objects for abstract network graph plot
    end

    methods (Access = public)
        function [obj, f] = initialize(obj, domain, objective, agents, timestep, maxIter, obstacles)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                domain (1, 1) {mustBeGeometry};
                objective (1, 1) {mustBeA(objective, 'sensingObjective')};
                agents (:, 1) cell {mustBeAgents};
                timestep (:, 1) double = 0.05;
                maxIter (:, 1) double = 1000;
                obstacles (:, 1) cell {mustBeGeometry} = cell(0, 1);
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
            end

            % Define simulation time parameters
            obj.timestep = timestep;
            obj.maxIter = maxIter;

            % Define domain
            obj.domain = domain;

            % Add geometries representing obstacles within the domain
            obj.obstacles = obstacles;

            % Define objective
            obj.objective = objective;

            % Define agents
            obj.agents = agents;

            % Compute adjacency matrix
            obj = obj.updateAdjacency();

            % Set up initial plot
            % Set up axes arrangement
            % Plot domain
            [obj.domain, f] = obj.domain.plotWireframe();

            % Plot obstacles
            for ii = 1:size(obj.obstacles, 1)
                [obj.obstacles{ii}, f] = obj.obstacles{ii}.plotWireframe(f);
            end

            % Plot objective gradient
            f = obj.objective.plot(f);

            % Plot agents and their collision geometries
            for ii = 1:size(obj.agents, 1)
                [obj.agents{ii}, f] = obj.agents{ii}.plot(f);
            end

            % Plot communication links
            [obj, f] = obj.plotConnections(f);

            % Plot abstract network graph
            [obj, f] = obj.plotGraph(f);
        end
        function [obj, f] = run(obj, f)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
            end

            % Create axes if they don't already exist
            f = firstPlotSetup(f);

            % Set up times to iterate over
            times = linspace(0, obj.timestep * obj.maxIter, obj.maxIter+1)';

            % Start video writer
            obj.v.FrameRate = 1/obj.timestep;
            obj.v.Quality = 90;
            obj.v.open();

            for ii = 1:size(times, 1)
                % Display current sim time
                t = times(ii);
                fprintf("Sim Time: %4.2f (%d/%d)\n", t, ii, obj.maxIter)

                % Iterate over agents to simulate their motion
                for jj = 1:size(obj.agents, 1)
                    obj.agents{jj} = obj.agents{jj}.run(obj.objective.objectiveFunction, obj.domain);
                end
    
                % Update adjacency matrix
                obj = obj.updateAdjacency;
    
                % Update plots
                [obj, f] = obj.updatePlots(f);

                % Write frame in to video
                I = getframe(f);
                obj.v.writeVideo(I);
            end

            % Close video file
            obj.v.close();
        end
        function [obj, f] = updatePlots(obj, f)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
            end

            % Update agent positions, collision geometries
            for ii = 1:size(obj.agents, 1)
                obj.agents{ii}.updatePlots();
            end

            % The remaining updates might be possible to do in a clever way
            % that moves existing lines instead of clearing and 
            % re-plotting, which is much better for performance boost

            % Update agent connections plot
            delete(obj.connectionsPlot);
            [obj, f] = obj.plotConnections(f);

            % Update network graph plot
            delete(obj.graphPlot);
            [obj, f] = obj.plotGraph(f);

            drawnow;
        end
        function obj = updateAdjacency(obj)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'miSim')};
            end

            % Initialize assuming only self-connections
            A = logical(eye(size(obj.agents, 1)));

            % Check lower triangle off-diagonal connections
            for ii = 2:size(A, 1)
                for jj = 1:(ii - 1)
                    if norm(obj.agents{ii}.pos - obj.agents{jj}.pos) <= min([obj.agents{ii}.comRange, obj.agents{jj}.comRange])
                        % Make sure that obstacles don't obstruct the line
                        % of sight, breaking the connection
                        for kk = 1:size(obj.obstacles, 1)
                            if ~obj.obstacles{kk}.containsLine(obj.agents{ii}.pos, obj.agents{jj}.pos)
                                A(ii, jj) = true;
                            end
                        end
                    end
                end
            end

            obj.adjacency = A | A';
        end
        function [obj, f] = plotConnections(obj, f)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
            end

            % Iterate over lower triangle off-diagonal region of the 
            % adjacency matrix to plot communications links between agents
            X = []; Y = []; Z = [];
            for ii = 2:size(obj.adjacency, 1)
                for jj = 1:(ii - 1)
                    if obj.adjacency(ii, jj)
                        X = [X; obj.agents{ii}.pos(1), obj.agents{jj}.pos(1)];
                        Y = [Y; obj.agents{ii}.pos(2), obj.agents{jj}.pos(2)];
                        Z = [Z; obj.agents{ii}.pos(3), obj.agents{jj}.pos(3)];
                    end
                end
            end
            X = X'; Y = Y'; Z = Z';

            % Plot the connections
            hold(f.CurrentAxes, "on");
            o = plot3(X, Y, Z, 'Color', 'g', 'LineWidth', 2, 'LineStyle', '--');
            hold(f.CurrentAxes, "off");

            % Check if this is a tiled layout figure
            if strcmp(f.Children(1).Type, 'tiledlayout')
                % Add to other plots
                o = [o, copyobj(o(:, 1), f.Children(1).Children(2))];
                o = [o, copyobj(o(:, 1), f.Children(1).Children(3))];
                o = [o, copyobj(o(:, 1), f.Children(1).Children(5))];
            end

            obj.connectionsPlot = o;
        end
        function [obj, f] = plotGraph(obj, f)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
            end

            % Form graph from adjacency matrix
            G = graph(obj.adjacency, 'omitselfloops');

            % Plot graph object
            hold(f.Children(1).Children(4), 'on');
            obj.graphPlot = plot(f.Children(1).Children(4), G, 'LineStyle', '--', 'EdgeColor', 'g', 'NodeColor', 'k', 'LineWidth', 2);
            hold(f.Children(1).Children(4), 'off');
        end
    end

    methods (Access = private)
        function validateInitialization(obj)
            % Assert obstacles do not intersect with the domain

            % Assert obstacles do not intersect with each other

            % Assert the objective has only one maxima within the domain

            % Assert the objective's sole maximum is not inaccessible due
            % to the placement of an obstacle

        end
        function validateLoop(obj)
            % Assert that agents are safely inside the domain

            % Assert that agents are not in proximity to obstacles

            % Assert that agents are not in proximity to each other

            % Assert that agents form a connected graph


        end
    end
end