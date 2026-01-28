function obj = plotH(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    obj.hf = figure;
    tiledlayout(obj.hf, 4, 1, "TileSpacing", "tight", "Padding", "compact");

    nexttile(obj.hf.Children(1));
    axes(obj.hf.Children(1).Children(1));
    grid(obj.hf.Children(1).Children(1), "on");
    xlabel(obj.hf.Children(1).Children(1), "Time (s)");
    title(obj.hf.Children(1).Children(1), "Collision Avoidance");
    hold(obj.hf.Children(1).Children(1), "on");
    obj.caPlot = plot(obj.h(1:(size(obj.agents, 1) * (size(obj.agents, 1) - 1) / 2), :)');
    legendStrings = [];
    for ii = 2:size(obj.agents, 1)
        for jj = 1:(ii - 1)
            legendStrings = [legendStrings; sprintf("A%d A%d", jj, ii)];
        end
    end
    legend(obj.hf.Children(1).Children(1), legendStrings, "Location", "bestoutside");
    hold(obj.hf.Children(1).Children(2), "off");

    nexttile(obj.hf.Children(1));
    axes(obj.hf.Children(1).Children(1));
    grid(obj.hf.Children(1).Children(1), "on");
    xlabel(obj.hf.Children(1).Children(1), "Time (s)");
    title(obj.hf.Children(1).Children(1), "Obstacles");
    hold(obj.hf.Children(1).Children(1), "on");
    obj.obsPlot = plot(obj.h((1 + (size(obj.agents, 1) * (size(obj.agents, 1) - 1) / 2)):(((size(obj.agents, 1) * (size(obj.agents, 1) - 1) / 2)) + size(obj.agents, 1) * size(obj.obstacles, 1)), :)');
    legendStrings = [];
    for ii = 1:size(obj.obstacles, 1)
        for jj = 1:size(obj.agents, 1)
            legendStrings = [legendStrings; sprintf("A%d O%d", jj, ii)];
        end
    end
    legend(obj.hf.Children(1).Children(1), legendStrings, "Location", "bestoutside");
    hold(obj.hf.Children(1).Children(2), "off");

    nexttile(obj.hf.Children(1));
    axes(obj.hf.Children(1).Children(1));
    grid(obj.hf.Children(1).Children(1), "on");
    xlabel(obj.hf.Children(1).Children(1), "Time (s)");
    title(obj.hf.Children(1).Children(1), "Domain");
    hold(obj.hf.Children(1).Children(1), "on");
    obj.domPlot = plot(obj.h((1 + (((size(obj.agents, 1) * (size(obj.agents, 1) - 1) / 2)) + size(obj.agents, 1) * size(obj.obstacles, 1))):size(obj.h, 1), 1:end)');
    legend(obj.hf.Children(1).Children(1), ["X Min"; "X Max"; "Y Min"; "Y Max"; "Z Min"; "Z Max";], "Location", "bestoutside");
    hold(obj.hf.Children(1).Children(2), "off");

    nexttile(obj.hf.Children(1));
    axes(obj.hf.Children(1).Children(1));
    grid(obj.hf.Children(1).Children(1), "on");
    xlabel(obj.hf.Children(1).Children(1), "Time (s)");
    title(obj.hf.Children(1).Children(1), "Communications");
    % skipped this for now because it is very complicated

end