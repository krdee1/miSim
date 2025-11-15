function [obj, f] = plot(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
    end

    % Plot domain
    [obj.domain, f] = obj.domain.plotWireframe(obj.spatialPlotIndices);

    % Plot obstacles
    for ii = 1:size(obj.obstacles, 1)
        [obj.obstacles{ii}, f] = obj.obstacles{ii}.plotWireframe(obj.spatialPlotIndices, f);
    end

    % Plot objective gradient
    f = obj.objective.plot(obj.objectivePlotIndices, f);

    % Plot agents and their collision geometries
    for ii = 1:size(obj.agents, 1)
        [obj.agents{ii}, f] = obj.agents{ii}.plot(obj.spatialPlotIndices, f);
    end

    % Plot communication links
    [obj, f] = obj.plotConnections(obj.spatialPlotIndices, f);

    % Plot abstract network graph
    [obj, f] = obj.plotGraph(obj.networkGraphIndex, f);

    % Plot domain partitioning
    [obj, f] = obj.plotPartitions(obj.partitionGraphIndex, f);

    % Enforce plot limits
    for ii = 1:size(obj.spatialPlotIndices, 2)
        xlim(f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(1), obj.domain.maxCorner(1)]);
        ylim(f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(2), obj.domain.maxCorner(2)]);
        zlim(f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(3), obj.domain.maxCorner(3)]);
    end
end