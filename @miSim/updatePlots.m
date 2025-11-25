function [obj] = updatePlots(obj, updatePartitions)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        updatePartitions (1, 1) logical = false;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
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
    obj = obj.plotConnections();

    % Update network graph plot
    delete(obj.graphPlot);
    obj = obj.plotGraph();

    % Update partitioning plot
    if updatePartitions
        delete(obj.partitionPlot);
        obj = obj.plotPartitions();
    end

    % reset plot limits to fit domain
    for ii = 1:size(obj.spatialPlotIndices, 2)
        xlim(obj.f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(1), obj.domain.maxCorner(1)]);
        ylim(obj.f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(2), obj.domain.maxCorner(2)]);
        zlim(obj.f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(3), obj.domain.maxCorner(3)]);
    end
    drawnow;
    
    % Update performance plot
    if updatePartitions
        % find index corresponding to the current time
        nowIdx = [0; obj.partitioningTimes] == obj.t;
        nowIdx = find(nowIdx);

        % Re-normalize performance plot
        normalizingFactor = 1/max(obj.perf(end, 1:nowIdx));
        obj.performancePlot(1).YData(1:nowIdx) = obj.perf(end, 1:nowIdx) * normalizingFactor;
        for ii = 2:size(obj.performancePlot, 1)
            obj.performancePlot(ii).YData(1:nowIdx) = obj.perf(ii - 1, 1:nowIdx) * normalizingFactor;
        end
    end
end