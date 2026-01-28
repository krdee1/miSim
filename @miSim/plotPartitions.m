function obj = plotPartitions(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    if isnan(obj.partitionGraphIndex)
        hold(obj.f.CurrentAxes, "on");
        o = imagesc(obj.f.CurrentAxes, obj.partitioning);
        hold(obj.f.CurrentAxes, "off");
    else
        hold(obj.f.Children(1).Children(obj.partitionGraphIndex(1)), "on");
        o = imagesc(obj.f.Children(1).Children(obj.partitionGraphIndex(1)), obj.partitioning);
        hold(obj.f.Children(1).Children(obj.partitionGraphIndex(1)), "off");
        if size(obj.partitionGraphIndex, 2) > 1
            for ii = 2:size(ind, 2)
                o = [o, copyobj(o(1), obj.f.Children(1).Children(obj.partitionGraphIndex(ii)))];
            end
        end
    end
    obj.partitionPlot = o;
end