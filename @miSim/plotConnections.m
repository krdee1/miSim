function [obj, f] = plotConnections(obj, ind, f)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        ind (1, :) double = NaN;
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
    end

    % Iterate over lower triangle off-diagonal region of the 
    % adjacency matrix to plot communications links between agents
    X = []; Y = []; Z = [];
    for ii = 2:size(obj.adjacency, 1)
        for jj = 1:(ii - 1)
            if obj.adjacency(ii, jj)
                X = [X; obj.agents{ii}.pos(1), obj.agents{jj}.pos(1)];
                Y = [Y; obj.agents{ii}.pos(2), obj.agents{jj}.pos(2)];
                Z = [Z; obj.agents{ii}.pos(3), obj.agents{jj}.pos(3)];
            end
        end
    end
    X = X'; Y = Y'; Z = Z';

    % Plot the connections
    if isnan(ind)
        hold(f.CurrentAxes, "on");
        o = plot3(f.CurrentAxes, X, Y, Z, 'Color', 'g', 'LineWidth', 2, 'LineStyle', '--');
        hold(f.CurrentAxes, "off");
    else
        hold(f.Children(1).Children(ind(1)), "on");
        o = plot3(f.Children(1).Children(ind(1)), X, Y, Z, 'Color', 'g', 'LineWidth', 2, 'LineStyle', '--');
        hold(f.Children(1).Children(ind(1)), "off");
    end

    % Copy to other plots
    if size(ind, 2) > 1
        for ii = 2:size(ind, 2)
            o = [o, copyobj(o(:, 1), f.Children(1).Children(ind(ii)))];
        end 
    end

    obj.connectionsPlot = o;
end