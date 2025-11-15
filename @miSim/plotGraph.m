function [obj, f] = plotGraph(obj, ind, f)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        ind (1, :) double = NaN;
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
    end

    % Form graph from adjacency matrix
    G = graph(obj.adjacency, 'omitselfloops');

    % Plot graph object
    if isnan(ind)
        hold(f.CurrentAxes, 'on');
        o = plot(f.CurrentAxes, G, 'LineStyle', '--', 'EdgeColor', 'g', 'NodeColor', 'k', 'LineWidth', 2);
        hold(f.CurrentAxes, 'off');
    else
        hold(f.Children(1).Children(ind(1)), 'on');
        o = plot(f.Children(1).Children(ind(1)), G, 'LineStyle', '--', 'EdgeColor', 'g', 'NodeColor', 'k', 'LineWidth', 2);
        hold(f.Children(1).Children(ind(1)), 'off');
        if size(ind, 2) > 1
            for ii = 2:size(ind, 2)
                o = [o; copyobj(o(1), f.Children(1).Children(ind(ii)))];
            end
        end
    end
    obj.graphPlot = o;
end