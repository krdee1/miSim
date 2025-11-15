function f = firstPlotSetup(f)
    arguments (Input)
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
    end
    arguments (Output)
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
    end
    if isempty(f.CurrentAxes)
        tiledlayout(f, 5, 5, "TileSpacing", "tight", "Padding", "compact");

        % 3D view
        nexttile(1, [4, 5]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "on");
        view(f.Children(1).Children(1), 3);
        xlabel(f.Children(1).Children(1), "X"); ylabel(f.Children(1).Children(1), "Y"); zlabel(f.Children(1).Children(1), "Z");
        title(f.Children(1).Children(1), "3D View");

        % Communications graph
        nexttile(21, [1, 1]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "off");
        view(f.Children(1).Children(1), 0, 90);
        title(f.Children(1).Children(1), "Network Graph");
        set(f.Children(1).Children(1), 'XTickLabelMode', 'manual');
        set(f.Children(1).Children(1), 'YTickLabelMode', 'manual');
        set(f.Children(1).Children(1), 'XTickLabel', {});
        set(f.Children(1).Children(1), 'YTickLabel', {});
        set(f.Children(1).Children(1), 'XTick', []);
        set(f.Children(1).Children(1), 'YTick', []);
        set(f.Children(1).Children(1), 'XColor', 'none');
        set(f.Children(1).Children(1), 'YColor', 'none');
        
        % Top-down view
        nexttile(22, [1, 1]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "on");
        view(f.Children(1).Children(1), 0, 90);
        xlabel(f.Children(1).Children(1), "X"); ylabel(f.Children(1).Children(1), "Y");
        title(f.Children(1).Children(1), "Top-down View");

        % Side-on view
        nexttile(23, [1, 1]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "on");
        view(f.Children(1).Children(1), 90, 0);
        ylabel(f.Children(1).Children(1), "Y"); zlabel(f.Children(1).Children(1), "Z");
        title(f.Children(1).Children(1), "Side-on View");

        % Front-on view
        nexttile(24, [1, 1]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "on");
        view(f.Children(1).Children(1), 0, 0);
        xlabel(f.Children(1).Children(1), "X"); zlabel(f.Children(1).Children(1), "Z");
        title(f.Children(1).Children(1), "Front-on View");

        % Partitioning
        nexttile(25, [1, 1]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "on");
        view(f.Children(1).Children(1), 0, 90);
        xlabel(f.Children(1).Children(1), "X"); ylabel(f.Children(1).Children(1), "Y");
        title(f.Children(1).Children(1), "Domain Partitioning");
        set(f.Children(1).Children(1), 'XTickLabelMode', 'manual');
        set(f.Children(1).Children(1), 'YTickLabelMode', 'manual');
        set(f.Children(1).Children(1), 'XTickLabel', {});
        set(f.Children(1).Children(1), 'YTickLabel', {});
        set(f.Children(1).Children(1), 'XTick', []);
        set(f.Children(1).Children(1), 'YTick', []);
    end
end