function [obj, f] = plotPartitions(obj, ind, f)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        ind (1, :) double = NaN;
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
    end

    if isnan(ind)
        hold(f.CurrentAxes, 'on');
        o = imagesc(f.CurrentAxes, obj.partitioning);
        hold(f.CurrentAxes, 'off');
    else
        hold(f.Children(1).Children(ind(1)), 'on');
        o = imagesc(f.Children(1).Children(ind(1)), obj.partitioning);
        hold(f.Children(1).Children(ind(1)), 'on');
        if size(ind, 2) > 1
            for ii = 2:size(ind, 2)
                o = [o, copyobj(o(1), f.Children(1).Children(ind(ii)))];
            end
        end
    end
    obj.partitionPlot = o;
end