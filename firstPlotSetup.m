function f = firstPlotSetup(f)
    if isempty(f.CurrentAxes)
        tiledlayout(f, 4, 3, "TileSpacing", "tight", "Padding", "compact");

        % Top-down view
        nexttile(1, [1, 2]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "on");
        view(f.Children(1).Children(1), 0, 90);
        xlabel(f.Children(1).Children(1), "X"); ylabel(f.Children(1).Children(1), "Y");
        title(f.Children(1).Children(1), "Top-down Perspective");

        % Communications graph
        nexttile(3, [1, 1]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "off");
        view(f.Children(1).Children(1), 0, 0);
        title(f.Children(1).Children(1), "Network Graph");

        % 3D view
        nexttile(4, [2, 2]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "on");
        view(f.Children(1).Children(1), 3);
        xlabel(f.Children(1).Children(1), "X"); ylabel(f.Children(1).Children(1), "Y"); zlabel(f.Children(1).Children(1), "Z");
        title(f.Children(1).Children(1), "3D Perspective");

        % Side-on view
        nexttile(6, [2, 1]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "on");
        view(f.Children(1).Children(1), 90, 0);
        ylabel(f.Children(1).Children(1), "Y"); zlabel(f.Children(1).Children(1), "Z");
        title(f.Children(1).Children(1), "Side-on Perspective");

        % Front-on view
        nexttile(10, [1, 2]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "on");
        view(f.Children(1).Children(1), 0, 0);
        xlabel(f.Children(1).Children(1), "X"); zlabel(f.Children(1).Children(1), "Z");
        title(f.Children(1).Children(1), "Front-on Perspective");
    end
end