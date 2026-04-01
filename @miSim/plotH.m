function obj = plotH(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    nCA  = size(obj.agents, 1) * (size(obj.agents, 1) - 1) / 2;
    nObs = size(obj.agents, 1) * size(obj.obstacles, 1);
    nDom = size(obj.agents, 1) * 6;

    obj.hf = figure;
    tiledlayout(obj.hf, 4, 1, "TileSpacing", "tight", "Padding", "compact");

    nexttile(obj.hf.Children(1));
    axes(obj.hf.Children(1).Children(1));
    grid(obj.hf.Children(1).Children(1), "on");
    xlabel(obj.hf.Children(1).Children(1), "Time (s)");
    title(obj.hf.Children(1).Children(1), "Collision Avoidance");
    hold(obj.hf.Children(1).Children(1), "on");
    obj.caPlot = plot(obj.barriers(1:nCA, :)');
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
    obj.obsPlot = plot(obj.barriers((nCA + 1):(nCA + nObs), :)');
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
    obj.domPlot = plot(obj.barriers((nCA + nObs + 1):(nCA + nObs + nDom), :)');
    domLabels = ["X Min", "X Max", "Y Min", "Y Max", "Z Min", "Z Max"];
    legendStrings = strings(nDom, 1);
    for ii = 1:size(obj.agents, 1)
        legendStrings((ii - 1) * 6 + (1:6)) = sprintf("A%d ", ii) + domLabels;
    end
    legend(obj.hf.Children(1).Children(1), legendStrings, "Location", "bestoutside");
    hold(obj.hf.Children(1).Children(2), "off");

    nexttile(obj.hf.Children(1));
    axes(obj.hf.Children(1).Children(1));
    grid(obj.hf.Children(1).Children(1), "on");
    xlabel(obj.hf.Children(1).Children(1), "Time (s)");
    title(obj.hf.Children(1).Children(1), "Communications");
    % skipped this for now because it is very complicated

end