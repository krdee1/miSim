function [obj] = updatePlots(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
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
    delete(obj.partitionPlot);
    obj = obj.plotPartitions();

    % reset plot limits to fit domain
    for ii = 1:size(obj.spatialPlotIndices, 2)
        xlim(obj.f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(1), obj.domain.maxCorner(1)]);
        ylim(obj.f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(2), obj.domain.maxCorner(2)]);
        zlim(obj.f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(3), obj.domain.maxCorner(3)]);
    end

    % Update agent trails
    for jj = 1:size(obj.spatialPlotIndices, 2)
        for ii = 1:size(obj.agents, 1)
            obj.trailPlot((jj - 1) * size(obj.agents, 1) + ii).XData(obj.timestepIndex) = obj.posHist(ii, obj.timestepIndex, 1);
            obj.trailPlot((jj - 1) * size(obj.agents, 1) + ii).YData(obj.timestepIndex) = obj.posHist(ii, obj.timestepIndex, 2);
            obj.trailPlot((jj - 1) * size(obj.agents, 1) + ii).ZData(obj.timestepIndex) = obj.posHist(ii, obj.timestepIndex, 3);
        end
    end

    drawnow;
    
    % Update performance plot
    % Re-normalize performance plot
    normalizingFactor = 1/max(obj.performance);
    obj.performancePlot(1).YData(1:(length(obj.performance) + 1)) = [obj.performance, 0] * normalizingFactor;
    obj.performancePlot(1).XData([obj.timestepIndex, obj.timestepIndex + 1]) = [obj.t, obj.t + obj.timestep];
    for ii = 1:(size(obj.agents, 1))
        obj.performancePlot(ii + 1).YData(1:(length(obj.performance) + 1)) = [obj.agents{ii}.performance(1:length(obj.performance)), 0] * normalizingFactor;
        obj.performancePlot(ii + 1).XData([obj.timestepIndex, obj.timestepIndex + 1]) = [obj.t, obj.t + obj.timestep];
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