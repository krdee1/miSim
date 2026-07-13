function [obj] = run(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    if coder.target('MATLAB')
        % Start video writer
        if obj.makeVideo
            v = obj.setupVideoWriter();
            drawnow;
            v.open();
            % Capture reference frame size; used to resize frames that deviate
            % due to figure reflow during plot updates (e.g. in headless mode).
            I_ref = getframe(obj.f);
            v.writeVideo(I_ref);
            videoFrameSize = [size(I_ref.cdata, 2), size(I_ref.cdata, 1)];
        end
    end

    for ii = 1:size(obj.times, 1)
        % Display current sim time
        obj.t = obj.times(ii);
        obj.timestepIndex = ii;
        if coder.target('MATLAB')
            fprintf("Sim Time: %4.2f (%d/%d)\n", obj.t, ii, obj.maxIter + 1);

            % Validate current simulation configuration
            obj.validate();
        end

        % Clear RF sensor caches
        if isa(obj.agents{1}.sensorModel, "rfSensor")
            for ss = 1:size(obj.agents, 1)
                obj.agents{ss}.sensorModel = obj.agents{ss}.sensorModel.clearRssCache;
            end
        end

        % Update partitioning before moving (this one is strictly for
        % plotting purposes, the real partitioning is done by the agents)
        [obj.partitioning, obj.agents] = obj.agents{1}.partition(obj.agents, obj.domain.objective);

        % Determine desired communications links: lesser-neighbor gives the
        % low-level connectivity topology; routing mode overlays bulk links
        if ~obj.useFixedTopology
            obj = obj.lesserNeighbor();
            if coder.target('MATLAB') && obj.useRoutingTopology
                obj = obj.routeTopology();
            end
        end

        % Log constraint adjacency for this timestep
        if coder.target('MATLAB')
            obj.constraintAdjacencyHist(:, :, ii) = obj.constraintAdjacencyMatrix;
        end

        % Moving
        % Iterate over agents to simulate their unconstrained motion
        for jj = 1:size(obj.agents, 1)
            obj.agents{jj} = obj.agents{jj}.run(obj.domain, obj.partitioning, obj.timestepIndex, jj, obj.useDoubleIntegrator, obj.dampingCoeff, obj.timestep, obj.optimizeSensorPointing, obj.agents([1:(jj - 1), (jj + 1):size(obj.agents, 1)]));
        end

        % Adjust motion determined by unconstrained gradient ascent using
        % CBF constraints solved by QP
        obj = constrainMotion(obj);

        if coder.target('MATLAB')
            % Update agent position and velocity history arrays
            obj.posHist(1:size(obj.agents, 1), obj.timestepIndex + 1, 1:3) = reshape(cell2mat(cellfun(@(x) x.pos, obj.agents, "UniformOutput", false)), size(obj.agents, 1), 1, 3);
            obj.velHist(1:size(obj.agents, 1), obj.timestepIndex + 1, 1:3) = reshape(cell2mat(cellfun(@(x) x.vel, obj.agents, "UniformOutput", false)), size(obj.agents, 1), 1, 3);

            % Update total performance
            obj.performance = [obj.performance, sum(cellfun(@(x) x.performance(obj.timestepIndex+1), obj.agents))];

            % Update adjacency matrix
            obj = obj.updateAdjacency();

            % Update plots
            obj = obj.updatePlots();

            % Write frame in to video
            if obj.makeVideo
                I = getframe(obj.f);
                if size(I.cdata, 2) ~= videoFrameSize(1) || size(I.cdata, 1) ~= videoFrameSize(2)
                    I.cdata = imresize(I.cdata, [videoFrameSize(2), videoFrameSize(1)]);
                end
                v.writeVideo(I);
            end
        end
    end

    % Close video
    if coder.target('MATLAB')
        if obj.makeVideo
            % Close video file
            v.close();
        end
    end
    
end
