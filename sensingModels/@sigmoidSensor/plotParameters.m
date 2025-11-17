function f = plotParameters(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'sigmoidSensor')};
    end
    arguments (Output)
        f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
    end

    % Distance and tilt sample points
    d = 0:(obj.alphaDist / 100):(2*obj.alphaDist);
    t = -90:1:90;

    % Sample membership functions
    d_x = obj.distanceMembership(d);
    t_x = obj.tiltMembership(deg2rad(t));

    % Plot resultant sigmoid curves
    f = figure;
    tiledlayout(f, 2, 1, "TileSpacing", "tight", "Padding", "compact");

    % Distance
    nexttile(1, [1,  1]);
    grid(f.Children(1).Children(1), "on");
    title(f.Children(1).Children(1), "Distance Membership Sigmoid");
    xlabel(f.Children(1).Children(1), "Distance (m)");
    ylabel(f.Children(1).Children(1), "Membership");
    hold(f.Children(1).Children(1), 'on');
    plot(f.Children(1).Children(1), d, d_x, 'LineWidth', 2);
    hold(f.Children(1).Children(1), 'off');
    ylim([0, 1]);

    % Tilt
    nexttile(2, [1,  1]);
    grid(f.Children(1).Children(1), "on");
    title(f.Children(1).Children(1), "Tilt Membership Sigmoid");
    xlabel(f.Children(1).Children(1), "Tilt (deg)");
    ylabel(f.Children(1).Children(1), "Membership");
    hold(f.Children(1).Children(1), 'on');
    plot(f.Children(1).Children(1), t, t_x, 'LineWidth', 2);
    hold(f.Children(1).Children(1), 'off');


    keyboard
end