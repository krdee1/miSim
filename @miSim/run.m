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

    steady = 0;
    for ii = 1:size(obj.times, 1)
        % Display current sim time
        obj.t = obj.times(ii);
        fprintf("Sim Time: %4.2f (%d/%d)\n", obj.t, ii, obj.maxIter + 1);

        % Check if it's time for new partitions
        updatePartitions = false;
        if ismember(obj.t, obj.partitioningTimes)
            % Check if it's time to end the sim (performance has settled)
            if obj.t >= obj.partitioningTimes(5)
                idx = find(obj.t == obj.partitioningTimes);
                newMeanTotalPerf = mean(obj.perf(end, ((idx - 5 + 1):idx)));
                if (obj.oldMeanTotalPerf * 0.95 <= newMeanTotalPerf) && (newMeanTotalPerf <= max(1e-6, obj.oldMeanTotalPerf * 1.05))
                    steady = steady + 1;
                    if steady >= 3
                        fprintf("Performance is stable, terminating early at %4.2f (%d/%d)\n", obj.t, ii, obj.maxIter + 1);
                        break; % performance is not improving further, exit main sim loop
                    end
                end
                obj.oldMeanTotalPerf = newMeanTotalPerf;
            end
            updatePartitions = true;
            obj = obj.partition();
        end

        % Iterate over agents to simulate their motion
        for jj = 1:size(obj.agents, 1)
            obj.agents{jj} = obj.agents{jj}.run(obj.objective, obj.domain, obj.partitioning);
        end

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