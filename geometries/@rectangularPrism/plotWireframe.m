function [obj, f] = plotWireframe(obj, ind, f)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rectangularPrism")};
        ind (1, :) double = NaN;
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")} = figure;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "rectangularPrism")};
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")};
    end

    % Create axes if they don't already exist
    f = firstPlotSetup(f);

    % Create plotting inputs from vertices and edges
    X = [obj.vertices(obj.edges(:,1),1), obj.vertices(obj.edges(:,2),1)]';
    Y = [obj.vertices(obj.edges(:,1),2), obj.vertices(obj.edges(:,2),2)]';
    Z = [obj.vertices(obj.edges(:,1),3), obj.vertices(obj.edges(:,2),3)]';

    % Plot the boundaries of the geometry into 3D view
    if isnan(ind)
        o = plot3(f.CurrentAxes, X, Y, Z, "-", "Color", obj.tag.color, "LineWidth", 2);
    else
        hold(f.Children(1).Children(ind(1)), "on");
        o = plot3(f.Children(1).Children(ind(1)), X, Y, Z, "-", "Color", obj.tag.color, "LineWidth", 2);
        hold(f.Children(1).Children(ind(1)), "off");
    end

    % Copy to other requested tiles
    if numel(ind) > 1
        for ii = 2:size(ind, 2)
            o = [o, copyobj(o(:, 1), f.Children(1).Children(ind(ii)))];
        end
    end

    obj.lines = o;
end