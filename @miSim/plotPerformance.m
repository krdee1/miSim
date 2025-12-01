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
    o.XData = NaN(size(o.XData)); % correct time will be set at runtime
    hold(obj.fPerf.Children(1), 'off');

    % Plot current agent performance
    for ii = 1:(size(obj.perf, 1) - 1)
        hold(obj.fPerf.Children(1), 'on');
        o = [o; plot(obj.fPerf.Children(1), obj.perf(ii, :))];
        o(end).XData = NaN(size(o(end).XData)); % correct time will be set at runtime
        hold(obj.fPerf.Children(1), 'off');
    end

    % Add legend
    agentStrings = repmat("Agent %d", size(obj.perf, 1) - 1, 1);
    for ii = 1:size(agentStrings, 1)
        agentStrings(ii) = sprintf(agentStrings(ii), ii);
    end
    agentStrings = ["Total"; agentStrings];
    legend(obj.fPerf.Children(1), agentStrings, 'Location', 'northwest');

    obj.performancePlot = o;
end