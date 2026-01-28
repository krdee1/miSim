function obj = plotConnections(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    % Iterate over lower triangle off-diagonal region of the 
    % adjacency matrix to plot communications links between agents
    X = []; Y = []; Z = [];
    for ii = 2:size(obj.constraintAdjacencyMatrix, 1)
        for jj = 1:(ii - 1)
            if obj.constraintAdjacencyMatrix(ii, jj)
                X = [X; obj.agents{ii}.pos(1), obj.agents{jj}.pos(1)];
                Y = [Y; obj.agents{ii}.pos(2), obj.agents{jj}.pos(2)];
                Z = [Z; obj.agents{ii}.pos(3), obj.agents{jj}.pos(3)];
            end
        end
    end
    X = X'; Y = Y'; Z = Z';

    % Plot the connections
    if isnan(obj.spatialPlotIndices)
        hold(obj.f.CurrentAxes, "on");
        o = plot3(obj.f.CurrentAxes, X, Y, Z, "Color", "g", "LineWidth", 2, "LineStyle", "--");
        hold(obj.f.CurrentAxes, "off");
    else
        hold(obj.f.Children(1).Children(obj.spatialPlotIndices(1)), "on");
        o = plot3(obj.f.Children(1).Children(obj.spatialPlotIndices(1)), X, Y, Z, "Color", "g", "LineWidth", 2, "LineStyle", "--");
        hold(obj.f.Children(1).Children(obj.spatialPlotIndices(1)), "off");
    end

    % Copy to other plots
    for ii = 2:size(obj.spatialPlotIndices, 2)
        o = [o, copyobj(o(:, 1), obj.f.Children(1).Children(obj.spatialPlotIndices(ii)))];
    end

    obj.connectionsPlot = o;
end