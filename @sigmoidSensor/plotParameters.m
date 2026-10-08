function f = plotParameters(obj, f)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "sigmoidSensor")};
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")} = figure;
    end
    arguments (Output)
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")};
    end

    % Distance and tilt sample points
    d = 0:(obj.alphaDist / 200):(2*obj.alphaDist);
    t = -90:0.5:90;

    % Sample membership functions
    d_x = obj.distanceMembership(d);
    t_x = obj.tiltMembership(t);

    % pad ends of sampled results with zeros at +/- some big number
    big = 1e6 * max(d);
    d = [-big, d, big];
    t = [-big, t, big];
    d_x = [0, d_x', 0];
    t_x = [0, t_x', 0];

    % Plot resultant sigmoid curves
    hold(gca(f), "on");
    if isempty(gca(f).Children)
        tiledlayout(f, 2, 1, "TileSpacing", "tight", "Padding", "compact");
    end

    % Distance
    nexttile(1, [1,  1]);
    grid("on");
    title("Distance Membership Sigmoid");
    xlabel("Distance (m)");
    ylabel("Membership");
    ld = legend();
    hold("on");
    plot(d, d_x, "LineWidth", 2);
    hold("off");
    xData = cell2mat(arrayfun(@(x) x.XData(2:(end - 1)), f.Children.Children(end).Children, 'UniformOutput', false));
    xlim([min(xData, [], "all"), max(xData, [], "all")]);
    ylim([0, 1]);
    
    % Tilt
    nexttile(2, [1,  1]);
    grid("on");
    title("Tilt Membership Sigmoid");
    xlabel("Tilt (deg)");
    ylabel("Membership");
    lt = legend();
    hold("on");
    plot(t, t_x, "LineWidth", 2);
    hold("off");
    xlim([-90, 90]);
    ylim([0, 1]);

    % add legends
    nexttile(1, [1,  1]);
    f.Children.Children(3).String{end} = sprintf("Sensor %d", length(f.Children.Children(end).Children));
    f.Children.Children(1).String{end} = sprintf("Sensor %d", length(f.Children.Children(end).Children));

    nexttile(2, [1,  1]);
    f.Children.Children(3).String{end} = sprintf("Sensor %d", length(f.Children.Children(end).Children));
    f.Children.Children(1).String{end} = sprintf("Sensor %d", length(f.Children.Children(end).Children));

    hold(gca(f), "off");
end