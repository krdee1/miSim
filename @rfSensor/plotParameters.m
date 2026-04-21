function f = plotParameters(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
    end
    arguments (Output)
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")};
    end
    
    % Distance and tilt sample points
    d_values = [0.01, 0.1, 0.25, 0.5, 0.75, 1, 2, 3, 4, 5:5:100];
    % t = zeros(size(d));
    t_values = -90:15:90;

    % Make grid of values of distances and tilts
    [d_mg, t_mg] = meshgrid(d_values, t_values);
    d = d_mg(:); t = t_mg(:); % flatten

    % Sample SINR (SNR) function by distances, tilts
    % using SINR method with no other transmitters defined is equivalent to SNR
    s_x = obj.sensorPerformance(d, t); % don't define other sensors

    % Plot resultant sigmoid curves
    f = figure;
    grid("on");
    surf(d_mg, t_mg, reshape(s_x, size(d_mg)));
    title("SNR vs Distance and Tilt");
    xlabel("Distance (m)");
    ylabel("Tilt (deg)");
    zlabel("SNR (dB)");
end