function obj = plot(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    
    % fast exit when plotting is disabled
    if ~obj.makePlots
        return;
    end

    % Plot domain
    [obj.domain, obj.f] = obj.domain.plotWireframe(obj.spatialPlotIndices);

    % Plot obstacles
    for ii = 1:size(obj.obstacles, 1)
        [obj.obstacles{ii}, obj.f] = obj.obstacles{ii}.plotWireframe(obj.spatialPlotIndices, obj.f);
    end

    % Plot objective gradient
    obj.f = obj.domain.objective.plot(obj.objectivePlotIndices, obj.f);

    % Plot agents and their collision geometries
    for ii = 1:size(obj.agents, 1)
        [obj.agents{ii}, obj.f] = obj.agents{ii}.plot(obj.spatialPlotIndices, obj.f);
    end

    % Plot communication links
    obj = obj.plotConnections();

    % Plot abstract network graph
    obj = obj.plotGraph();

    % Plot domain partitioning
    obj = obj.plotPartitions();

    % Enforce plot limits
    for ii = 1:size(obj.spatialPlotIndices, 2)
        xlim(obj.f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(1), obj.domain.maxCorner(1)]);
        ylim(obj.f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(2), obj.domain.maxCorner(2)]);
        zlim(obj.f.Children(1).Children(obj.spatialPlotIndices(ii)), [obj.domain.minCorner(3), obj.domain.maxCorner(3)]);
    end

    % Plot performance
    obj = obj.plotPerformance();
end