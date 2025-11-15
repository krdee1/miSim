function [obj, f] = plot(obj, ind, f)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'cone')};
        ind (1, :) double = NaN;
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'cone')};
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
    end
    
    % Create axes if they don't already exist
    f = firstPlotSetup(f);
    
    % Plot cone
    [X, Y, Z] = cylinder([obj.radius, 0], obj.n);
    
    % Scale to match height
    Z = Z * obj.height;
    
    % Move to center location
    X = X + obj.center(1);
    Y = Y + obj.center(2);
    Z = Z + obj.center(3);
    
    % Plot
    if isnan(ind)
        o = surf(f.CurrentAxes, X, Y, Z);
    else
        hold(f.Children(1).Children(ind(1)), "on");
        o = surf(f.Children(1).Children(ind(1)), X, Y, Z, ones([size(Z), 1]) .* reshape(obj.tag.color, 1, 1, 3), 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        hold(f.Children(1).Children(ind(1)), "off");
    end
    
    % Copy to other requested tiles
    if numel(ind) > 1
        for ii = 2:size(ind, 2)
            o = [o, copyobj(o(:, 1), f.Children(1).Children(ind(ii)))];
        end
    end
    
    obj.surface = o;
end