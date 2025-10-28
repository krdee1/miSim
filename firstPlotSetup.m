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
        title("Top-down Perspective");

        % Communications graph
        nexttile(3, [1, 1]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "off");
        view(f.Children(1).Children(1), 0, 0);
        title("Network Graph");

        % 3D view
        title("3D Perspective");
        nexttile(4, [2, 2]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "on");
        view(f.Children(1).Children(1), 3);
        xlabel(f.Children(1).Children(1), "X"); ylabel(f.Children(1).Children(1), "Y"); zlabel(f.Children(1).Children(1), "Z");

        % Side-on view
        title("Side-on Perspective");
        nexttile(6, [2, 1]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "on");
        view(f.Children(1).Children(1), 90, 0);
        ylabel(f.Children(1).Children(1), "Y"); zlabel(f.Children(1).Children(1), "Z");

        % Front-on view
        title("Front-on Perspective");
        nexttile(10, [1, 2]);
        axes(f.Children(1).Children(1));
        axis(f.Children(1).Children(1), "image");
        grid(f.Children(1).Children(1), "on");
        view(f.Children(1).Children(1), 0, 0);
        xlabel(f.Children(1).Children(1), "X"); zlabel(f.Children(1).Children(1), "Z");
    end
end