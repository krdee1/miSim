function [obj] = updatePlots(obj, updatePartitions)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        updatePartitions (1, 1) logical = false;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end

    % Fast exit when plotting is disabled
    if ~obj.makePlots
        return;
    end

    % Update agent positions, collision/communication geometries
    for ii = 1:size(obj.agents, 1)
        obj.agents{ii}.updatePlots();
    end

    % The remaining updates might should all be possible to do in a clever 
    % way that moves existing lines instead of clearing and 
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

    % Update agent trails
    for ii = 1:size(obj.agents, 1)
        obj.trailPlot(ii).XData(obj.timestepIndex) = obj.posHist(ii, obj.timestepIndex, 1);
        obj.trailPlot(ii).YData(obj.timestepIndex) = obj.posHist(ii, obj.timestepIndex, 2);
        obj.trailPlot(ii).ZData(obj.timestepIndex) = obj.posHist(ii, obj.timestepIndex, 3);
    end

    drawnow;
    
    % Update performance plot
    % Re-normalize performance plot
    normalizingFactor = 1/max(obj.performance(end));
    obj.performancePlot(1).YData(1:length(obj.performance)) = obj.performance * normalizingFactor;
    obj.performancePlot(1).XData(obj.timestepIndex) = obj.t;
    for ii = 2:(size(obj.agents, 1) + 1)
        obj.performancePlot(ii).YData(1:length(obj.performance)) = obj.agents{ii - 1}.performance * normalizingFactor;
        obj.performancePlot(ii).XData(obj.timestepIndex) = obj.t;
    end

    % Update h function plots
    for ii = 1:size(obj.caPlot, 1)
        obj.caPlot(ii).YData(obj.timestepIndex) = obj.h(ii, obj.timestepIndex);
    end
    for ii = 1:size(obj.obsPlot, 1)
        obj.obsPlot(ii).YData(obj.timestepIndex) = obj.h(ii + size(obj.caPlot, 1), obj.timestepIndex);
    end
    for ii = 1:size(obj.domPlot, 1)
        obj.domPlot(ii).YData(obj.timestepIndex) = obj.h(ii + size(obj.caPlot, 1) + size(obj.obsPlot, 1), obj.timestepIndex);
    end
end