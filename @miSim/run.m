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
    v = obj.setupVideoWriter();
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