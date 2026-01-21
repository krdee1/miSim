function obj = teardown(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end

    % Close plots
    close(obj.hf);
    close(obj.fPerf);
    close(obj.f);

    % Reset accumulators
    obj.performance = 0;

    % Reset agents
    for ii = 1:size(obj.agents, 1)
        obj.agents{ii} = agent;
    end


end