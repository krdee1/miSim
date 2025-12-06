function [obj] = run(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end

    % Start video writer
    v = obj.setupVideoWriter();
    v.open();
    
    for ii = 1:size(obj.times, 1)
        % Display current sim time
        obj.t = obj.times(ii);
        fprintf("Sim Time: %4.2f (%d/%d)\n", obj.t, ii, obj.maxIter + 1);

        % Check if it's time for new partitions
        updatePartitions = false;
        if ismember(obj.t, obj.partitioningTimes)
            updatePartitions = true;
            obj = obj.partition();
        end

        % Iterate over agents to simulate their motion
        for jj = 1:size(obj.agents, 1)
            obj.agents{jj} = obj.agents{jj}.run(obj.domain, obj.partitioning, obj.t);
        end

        % Adjust motion determined by unconstrained gradient ascent using
        % CBF constraints solved by QP
        obj = constrainMotion(obj);

        % Update total performance
        obj.performance = [obj.performance, sum(cellfun(@(x) x.performance(end), obj.agents))];

        % Update adjacency matrix
        obj = obj.updateAdjacency();

        % Update plots
        obj = obj.updatePlots(updatePartitions);

        % Write frame in to video
        I = getframe(obj.f);
        v.writeVideo(I);
    end

    % Close video file
    v.close();
end
