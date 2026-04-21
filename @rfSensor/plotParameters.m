function f = plotParameters(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
    end
    arguments (Output)
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")};
    end

    % Distance and tilt sample points
    d = [0.01, 0.1, 0.25, 0.5, 0.75, 1:1:100];
    t  = zeros(size(d));

    % Sample RSS function by distances, tilts
    r_x = obj.RSS(d', t');

    % Sample SINR (SNR) function by distances, tilts
    % using SINR method with no other transmitters defined is equivalent to SNR
    s_x = obj.sensorPerformance(d, t); % don't define other sensors

    % Plot resultant sigmoid curves
    f = figure;
    tiledlayout(f, 2, 1, "TileSpacing", "tight", "Padding", "compact");

    % RSS/Distance with 0 tilt
    nexttile(1, [1,  1]);
    grid("on");
    title("RSS vs Distance");
    xlabel("Distance (m)");
    ylabel("RSS (dBm)");
    hold("on");
    plot(d, r_x, "LineWidth", 2);
    hold("off");
    % ylim([0, 1]);

    % SNR/Distance with 0 tilt
    nexttile(2, [1,  1]);
    grid("on");
    title("SNR vs Distance");
    xlabel("Distance (m)");
    ylabel("SNR (dB)");
    hold("on");
    plot(d, s_x, "LineWidth", 2);
    hold("off");
end