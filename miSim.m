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

    methods (Access = public)
        function obj = initialize(obj, domain, objective, agents, timestep, maxIter, obstacles)
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

        end
        function obj = run(obj)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'miSim')};
            end
            keyboard
            % Iterate over agents to simulate their motion
            for ii = 1:size(obj.agents, 1)
                obj.agents{ii}
            end

            % Update adjacency matrix
            obj = obj.updateAdjacency;

            % Update plots

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
        function f = plotNetwork(obj, f)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
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
                copyobj(o, f.Children(1).Children(2));
                copyobj(o, f.Children(1).Children(3));
                copyobj(o, f.Children(1).Children(5));
            end
        end
        function f = plotGraph(obj, f)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
            end

            % Form graph from adjacency matrix
            G = graph(obj.adjacency, 'omitselfloops');

            % Check if this is a tiled layout figure
            if strcmp(f.Children(1).Type, 'tiledlayout')
                o = plot(f.Children(1).Children(4), G, 'LineStyle', '--', 'EdgeColor', 'g', 'NodeColor', 'k', 'LineWidth', 2);
            else
                o = plot(f.CurrentAxes, G, 'LineStyle', '--', 'EdgeColor', 'g', 'NodeColor', 'k', 'LineWidth', 2);
            end
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