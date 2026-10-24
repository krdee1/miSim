function f = firstPlotSetup(f)
    arguments (Input)
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")} = figure;
    end
    arguments (Output)
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")};
    end
    if isempty(f.CurrentAxes)
        tiledlayout(f, 5, 5, "TileSpacing", "tight", "Padding", "compact");

        % 3D view
        ax = nexttile(1, [4, 5]);
        axis(ax, "image");
        grid(ax, "on");
        view(ax, 3);
        xlabel(ax, "X"); ylabel(ax, "Y"); zlabel(ax, "Z");
        title(ax, "3D View");

        % Communications graph
        ax = nexttile(21, [1, 1]);
        axis(ax, "image");
        grid(ax, "off");
        view(ax, 0, 90);
        title(ax, "Network Graph");
        set(ax, "XTickLabelMode", "manual");
        set(ax, "YTickLabelMode", "manual");
        set(ax, "XTickLabel", {});
        set(ax, "YTickLabel", {});
        set(ax, "XTick", []);
        set(ax, "YTick", []);
        set(ax, "XColor", "none");
        set(ax, "YColor", "none");

        % Top-down view
        ax = nexttile(22, [1, 1]);
        axis(ax, "image");
        grid(ax, "on");
        view(ax, 0, 90);
        xlabel(ax, "X"); ylabel(ax, "Y");
        title(ax, "Top-down View");

        % Side-on view
        ax = nexttile(23, [1, 1]);
        axis(ax, "image");
        grid(ax, "on");
        view(ax, 90, 0);
        ylabel(ax, "Y"); zlabel(ax, "Z");
        title(ax, "Side-on View");

        % Front-on view
        ax = nexttile(24, [1, 1]);
        axis(ax, "image");
        grid(ax, "on");
        view(ax, 0, 0);
        xlabel(ax, "X"); zlabel(ax, "Z");
        title(ax, "Front-on View");

        % Partitioning
        ax = nexttile(25, [1, 1]);
        axis(ax, "image");
        grid(ax, "on");
        view(ax, 0, 90);
        xlabel(ax, "X"); ylabel(ax, "Y");
        title(ax, "Domain Partitioning");
        set(ax, "XTickLabelMode", "manual");
        set(ax, "YTickLabelMode", "manual");
        set(ax, "XTickLabel", {});
        set(ax, "YTickLabel", {});
        set(ax, "XTick", []);
        set(ax, "YTick", []);
    end
end
