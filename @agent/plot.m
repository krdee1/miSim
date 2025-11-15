function [obj, f] = plot(obj, ind, f)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'agent')};
        ind (1, :) double = NaN;
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'agent')};
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
    end

    % Create axes if they don't already exist
    f = firstPlotSetup(f);

    % Plot points representing the agent position
    hold(f.Children(1).Children(end), "on");
    o = scatter3(f.Children(1).Children(end), obj.pos(1), obj.pos(2), obj.pos(3), 'filled', 'ko', 'SizeData', 25);
    hold(f.Children(1).Children(end), "off");

    % Check if this is a tiled layout figure
    if strcmp(f.Children(1).Type, 'tiledlayout')
        % Add to other perspectives
        o = [o; copyobj(o(1), f.Children(1).Children(2))];
        o = [o; copyobj(o(1), f.Children(1).Children(3))];
        o = [o; copyobj(o(1), f.Children(1).Children(4))];
    end

    obj.scatterPoints = o;

    % Plot collision geometry
    [obj.collisionGeometry, f] = obj.collisionGeometry.plotWireframe(ind, f);

    % Plot FOV geometry
    [obj.fovGeometry, f] = obj.fovGeometry.plot(ind, f);
end