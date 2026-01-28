function obj = plotGraph(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    % Form graph from adjacency matrix
    G = graph(obj.constraintAdjacencyMatrix, "omitselfloops");

    % Plot graph object
    if isnan(obj.networkGraphIndex)
        hold(obj.f.CurrentAxes, "on");
        o = plot(obj.f.CurrentAxes, G, "LineStyle", "--", "EdgeColor", "g", "NodeColor", "k", "LineWidth", 2);
        hold(obj.f.CurrentAxes, "off");
    else
        hold(obj.f.Children(1).Children(obj.networkGraphIndex(1)), "on");
        o = plot(obj.f.Children(1).Children(obj.networkGraphIndex(1)), G, "LineStyle", "--", "EdgeColor", "g", "NodeColor", "k", "LineWidth", 2);
        hold(obj.f.Children(1).Children(obj.networkGraphIndex(1)), "off");
        if size(obj.networkGraphIndex, 2) > 1
            for ii = 2:size(ind, 2)
                o = [o; copyobj(o(1), obj.f.Children(1).Children(obj.networkGraphIndex(ii)))];
            end
        end
    end
    obj.graphPlot = o;
end