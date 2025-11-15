function [obj, f] = updatePlots(obj, f, updatePartitions)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
        updatePartitions (1, 1) logical = false;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
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
    [obj, f] = obj.plotConnections(obj.spatialPlotIndices, f);

    % Update network graph plot
    delete(obj.graphPlot);
    [obj, f] = obj.plotGraph(obj.networkGraphIndex, f);

    % Update partitioning plot
    if updatePartitions
        delete(obj.partitionPlot);
        [obj, f] = obj.plotPartitions(obj.partitionGraphIndex, f);
    end

    % reset plot limits to fit domain
    for ii = 1:size(obj.spatialPlotIndices, 2)
        xlim(f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(1), obj.domain.maxCorner(1)]);
        ylim(f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(2), obj.domain.maxCorner(2)]);
        zlim(f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(3), obj.domain.maxCorner(3)]);
    end

    drawnow;
end