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

    normalized = obj.values ./ sum(obj.values, "all");
    cRange = [min(normalized, [], "all"), max(normalized, [], "all")];

    % Plot gradient on the "floor" of the domain
    if isnan(ind)
        ax = f.CurrentAxes;
        hold(ax, "on");
        o = surf(ax, obj.X, obj.Y, zeros(size(obj.X)), normalized, "EdgeColor", "none");
        o.HitTest = "off";
        o.PickableParts = "none";
        clim(ax, cRange);
        hold(ax, "off");
    else
        ax = f.Children(1).Children(ind(1));
        hold(ax, "on");
        o = surf(ax, obj.X, obj.Y, zeros(size(obj.X)), normalized, "EdgeColor", "none");
        o.HitTest = "off";
        o.PickableParts = "none";
        clim(ax, cRange);
        hold(ax, "off");
    end

    % Add to other perspectives
    if size(ind, 2) > 1
        for ii = 2:size(ind, 2)
            copyobj(o, f.Children(1).Children(ind(ii)));
        end
    end
end