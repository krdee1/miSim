function [obj] = run(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end

    % Start video writer
    if obj.makeVideo
        v = obj.setupVideoWriter();
        v.open();
    end
    
    for ii = 1:size(obj.times, 1)
        % Display current sim time
        obj.t = obj.times(ii);
        obj.timestepIndex = ii;
        fprintf("Sim Time: %4.2f (%d/%d)\n", obj.t, ii, obj.maxIter + 1);

        % Validate current simulation configuration
        obj.validate();

        % Check if it's time for new partitions
        updatePartitions = false;
        if ismember(obj.t, obj.partitioningTimes)
            updatePartitions = true;
            obj = obj.partition();
        end

        % Determine desired communications links
        obj = obj.lesserNeighbor();

        % Iterate over agents to simulate their unconstrained motion
        for jj = 1:size(obj.agents, 1)
            obj.agents{jj} = obj.agents{jj}.run(obj.domain, obj.partitioning, obj.t, jj);
        end

        % Adjust motion determined by unconstrained gradient ascent using
        % CBF constraints solved by QP
        obj = constrainMotion(obj);

        % Finished simulation for this timestep, do accounting

        % Update agent position history array
        obj.posHist(1:size(obj.agents, 1), obj.timestepIndex + 1, 1:3) = reshape(cell2mat(cellfun(@(x) x.pos, obj.agents, 'UniformOutput', false)), size(obj.agents, 1), 1, 3);

        % Update total performance
        obj.performance = [obj.performance, sum(cellfun(@(x) x.performance(end), obj.agents))];

        % Update adjacency matrix
        obj = obj.updateAdjacency();

        % Update plots
        obj = obj.updatePlots(updatePartitions);

        % Write frame in to video
        if obj.makeVideo
            I = getframe(obj.f);
            v.writeVideo(I);
        end
    end

    if obj.makeVideo
        % Close video file
        v.close();
    end
end
