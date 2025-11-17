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
    grid("on");
    title("Distance Membership Sigmoid");
    xlabel("Distance (m)");
    ylabel("Membership");
    hold('on');
    plot(d, d_x, 'LineWidth', 2);
    hold('off');
    ylim([0, 1]);

    % Tilt
    nexttile(2, [1,  1]);
    grid("on");
    title("Tilt Membership Sigmoid");
    xlabel("Tilt (deg)");
    ylabel("Membership");
    hold('on');
    plot(t, t_x, 'LineWidth', 2);
    hold('off');
    ylim([0, 1]);
end