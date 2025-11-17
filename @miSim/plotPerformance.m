function obj = plotPerformance(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end

    axes(obj.fPerf);
    title(obj.fPerf.Children(1), "Sensor Performance");
    xlabel(obj.fPerf.Children(1), 'Time (s)');
    ylabel(obj.fPerf.Children(1), 'Sensor Performance');
    grid(obj.fPerf.Children(1), 'on');

    % Plot current cumulative performance
    hold(obj.fPerf.Children(1), 'on');
    o = plot(obj.fPerf.Children(1), obj.perf(end, :));
    hold(obj.fPerf.Children(1), 'off');

    % Plot current agent performance
    for ii = 1:(size(obj.perf, 1) - 1)
        hold(obj.fPerf.Children(1), 'on');
        o = [o; plot(obj.fPerf.Children(1), obj.perf(ii, :))];
        hold(obj.fPerf.Children(1), 'off');
    end

    obj.performancePlot = o;
end