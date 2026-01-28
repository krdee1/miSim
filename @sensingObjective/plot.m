function f = plot(obj, ind, f)
    arguments (Input)
        obj (1,1) {mustBeA(obj, "sensingObjective")};
        ind (1, :) double = NaN;
        f (1,1) {mustBeA(f, "matlab.ui.Figure")} = figure;
    end
    arguments (Output)
        f (1,1) {mustBeA(f, "matlab.ui.Figure")};
    end

    % Create axes if they don't already exist
    f = firstPlotSetup(f);

    % Plot gradient on the "floor" of the domain
    if isnan(ind)
        hold(f.CurrentAxes, "on");
        o = surf(f.CurrentAxes, obj.X, obj.Y, zeros(size(obj.X)), obj.values ./ max(obj.values, [], "all"), "EdgeColor", "none");
        o.HitTest = "off";
        o.PickableParts = "none";
        hold(f.CurrentAxes, "off");
    else
        hold(f.Children(1).Children(ind(1)), "on");
        o = surf(f.Children(1).Children(ind(1)), obj.X, obj.Y, zeros(size(obj.X)), obj.values ./ max(obj.values, [], "all"), "EdgeColor", "none");
        o.HitTest = "off";
        o.PickableParts = "none";
        hold(f.Children(1).Children(ind(1)), "off");
    end

    % Add to other perspectives
    if size(ind, 2) > 1
        for ii = 2:size(ind, 2)
            copyobj(o, f.Children(1).Children(ind(ii)));
        end
    end
end