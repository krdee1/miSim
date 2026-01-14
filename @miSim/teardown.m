function teardown(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
    end

    % Close plots
    close(obj.hf);
    close(obj.fPerf);
    close(obj.f);

end