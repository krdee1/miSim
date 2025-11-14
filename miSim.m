classdef miSim
    % multiagent interconnection simulation

    % Simulation parameters
    properties (SetAccess = private, GetAccess = public)
        timestep = NaN; % delta time interval for simulation iterations
        partitioningFreq = NaN; % number of simulation timesteps at which the partitioning routine is re-run
        maxIter = NaN; % maximum number of simulation iterations
        domain = rectangularPrism;
        objective = sensingObjective;
        obstacles = cell(0, 1); % geometries that define obstacles within the domain
        agents = cell(0, 1); % agents that move within the domain
        adjacency = NaN; % Adjacency matrix representing communications network graph
        partitioning = NaN;
    end

    properties (Access = private)
        % Plot objects
        connectionsPlot; % objects for lines connecting agents in spatial plots
        graphPlot; % objects for abstract network graph plot
        partitionPlot; % objects for partition plot

        % Indicies for various plot types in the main tiled layout figure
        spatialPlotIndices = [6, 4, 3, 2];
        objectivePlotIndices = [6, 4];
        networkGraphIndex = 5;
        partitionGraphIndex = 1;
    end

    methods (Access = public)
        function [obj, f] = initialize(obj, domain, objective, agents, timestep, partitoningFreq, maxIter, obstacles)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                domain (1, 1) {mustBeGeometry};
                objective (1, 1) {mustBeA(objective, 'sensingObjective')};
                agents (:, 1) cell;
                timestep (:, 1) double = 0.05;
                partitoningFreq (:, 1) double = 0.25
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
            obj.partitioningFreq = partitoningFreq;

            % Add geometries representing obstacles within the domain
            obj.obstacles = obstacles;

            % Define objective
            obj.objective = objective;

            % Define agents
            obj.agents = agents;

            % Compute adjacency matrix
            obj = obj.updateAdjacency();

            % Create initial partitioning
            obj = obj.partition();

            % Set up initial plot
            % Set up axes arrangement
            % Plot domain
            [obj.domain, f] = obj.domain.plotWireframe(obj.spatialPlotIndices);

            % Plot obstacles
            for ii = 1:size(obj.obstacles, 1)
                [obj.obstacles{ii}, f] = obj.obstacles{ii}.plotWireframe(obj.spatialPlotIndices, f);
            end

            % Plot objective gradient
            f = obj.objective.plot(obj.objectivePlotIndices, f);

            % Plot agents and their collision geometries
            for ii = 1:size(obj.agents, 1)
                [obj.agents{ii}, f] = obj.agents{ii}.plot(obj.spatialPlotIndices, f);
            end

            % Plot communication links
            [obj, f] = obj.plotConnections(obj.spatialPlotIndices, f);

            % Plot abstract network graph
            [obj, f] = obj.plotGraph(obj.networkGraphIndex, f);

            % Plot domain partitioning
            [obj, f] = obj.plotPartitions(obj.partitionGraphIndex, f);

            % Enforce plot limits
            for ii = 1:size(obj.spatialPlotIndices, 2)
                xlim(f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(1), obj.domain.maxCorner(1)]);
                ylim(f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(2), obj.domain.maxCorner(2)]);
                zlim(f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(3), obj.domain.maxCorner(3)]);
            end
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
            partitioningTimes = times(obj.partitioningFreq:obj.partitioningFreq:size(times, 1));

            % Start video writer
            v = setupVideoWriter(obj.timestep);
            v.open();

            for ii = 1:size(times, 1)
                % Display current sim time
                t = times(ii);
                fprintf("Sim Time: %4.2f (%d/%d)\n", t, ii, obj.maxIter)

                % Check if it's time for new partitions
                updatePartitions = false;
                if ismember(t, partitioningTimes)
                    updatePartitions = true;
                    obj = obj.partition();
                end

                % Iterate over agents to simulate their motion
                for jj = 1:size(obj.agents, 1)
                    obj.agents{jj} = obj.agents{jj}.run(obj.objective, obj.domain, obj.partitioning);
                end
    
                % Update adjacency matrix
                obj = obj.updateAdjacency;
    
                % Update plots
                [obj, f] = obj.updatePlots(f, updatePartitions);

                % Write frame in to video
                I = getframe(f);
                v.writeVideo(I);
            end

            % Close video file
            v.close();
        end
        function obj = partition(obj)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'miSim')};
            end

            % Assess sensing performance of each agent at each sample point
            % in the domain
            agentPerformances = cellfun(@(x) reshape(x.sensorModel.sensorPerformance(x.pos, x.pan, x.tilt, [obj.objective.X(:), obj.objective.Y(:), zeros(size(obj.objective.X(:)))]), size(obj.objective.X)), obj.agents, 'UniformOutput', false);
            agentPerformances = cat(3, agentPerformances{:});
            
            % Get highest performance value at each point
            [~, idx] = max(agentPerformances, [], 3);

            % Collect agent indices in the same way
            agentInds = cellfun(@(x) x.index * ones(size(obj.objective.X)), obj.agents, 'UniformOutput', false);
            agentInds = cat(3, agentInds{:});

            % Get highest performing agent's index
            [m,n,~] = size(agentInds);
            [i,j] = ndgrid(1:m, 1:n);
            obj.partitioning = agentInds(sub2ind(size(agentInds), i, j, idx));
        end
        function [obj, f] = updatePlots(obj, f, updatePartitions)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
                updatePartitions (1, 1) logical = false;
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
            [obj, f] = obj.plotConnections(obj.spatialPlotIndices, f);

            % Update network graph plot
            delete(obj.graphPlot);
            [obj, f] = obj.plotGraph(obj.networkGraphIndex, f);

            % Update partitioning plot
            if updatePartitions
                delete(obj.partitionPlot);
                [obj, f] = obj.plotPartitions(obj.partitionGraphIndex, f);
            end

            % reset plot limits to fit domain
            for ii = 1:size(obj.spatialPlotIndices, 2)
                xlim(f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(1), obj.domain.maxCorner(1)]);
                ylim(f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(2), obj.domain.maxCorner(2)]);
                zlim(f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(3), obj.domain.maxCorner(3)]);
            end

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
                        % need extra handling for cases with no obstacles
                        if isempty(obj.obstacles)
                            A(ii, jj) = true;
                        end
                    end
                end
            end

            obj.adjacency = A | A';
        end
        function [obj, f] = plotConnections(obj, ind, f)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                ind (1, :) double = NaN;
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
            if isnan(ind)
                hold(f.CurrentAxes, "on");
                o = plot3(f.CurrentAxes, X, Y, Z, 'Color', 'g', 'LineWidth', 2, 'LineStyle', '--');
                hold(f.CurrentAxes, "off");
            else
                hold(f.Children(1).Children(ind(1)), "on");
                o = plot3(f.Children(1).Children(ind(1)), X, Y, Z, 'Color', 'g', 'LineWidth', 2, 'LineStyle', '--');
                hold(f.Children(1).Children(ind(1)), "off");
            end

            % Copy to other plots
            if size(ind, 2) > 1
                for ii = 2:size(ind, 2)
                    o = [o, copyobj(o(:, 1), f.Children(1).Children(ind(ii)))];
                end 
            end

            obj.connectionsPlot = o;
        end
        function [obj, f] = plotPartitions(obj, ind, f)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                ind (1, :) double = NaN;
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
            end

            if isnan(ind)
                hold(f.CurrentAxes, 'on');
                o = imagesc(f.CurrentAxes, obj.partitioning);
                hold(f.CurrentAxes, 'off');
            else
                hold(f.Children(1).Children(ind(1)), 'on');
                o = imagesc(f.Children(1).Children(ind(1)), obj.partitioning);
                hold(f.Children(1).Children(ind(1)), 'on');
                if size(ind, 2) > 1
                    for ii = 2:size(ind, 2)
                        o = [o, copyobj(o(1), f.Children(1).Children(ind(ii)))];
                    end
                end
            end
            obj.partitionPlot = o;

        end
        function [obj, f] = plotGraph(obj, ind, f)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                ind (1, :) double = NaN;
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
            end

            % Form graph from adjacency matrix
            G = graph(obj.adjacency, 'omitselfloops');

            % Plot graph object
            if isnan(ind)
                hold(f.CurrentAxes, 'on');
                o = plot(f.CurrentAxes, G, 'LineStyle', '--', 'EdgeColor', 'g', 'NodeColor', 'k', 'LineWidth', 2);
                hold(f.CurrentAxes, 'off');
            else
                hold(f.Children(1).Children(ind(1)), 'on');
                o = plot(f.Children(1).Children(ind(1)), G, 'LineStyle', '--', 'EdgeColor', 'g', 'NodeColor', 'k', 'LineWidth', 2);
                hold(f.Children(1).Children(ind(1)), 'off');
                if size(ind, 2) > 1
                    for ii = 2:size(ind, 2)
                        o = [o; copyobj(o(1), f.Children(1).Children(ind(ii)))];
                    end
                end
            end
            obj.graphPlot = o;
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