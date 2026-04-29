function value = transmitterGain(obj, t, a)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        t (:, 1) double; % LOS tilt angle
        a (:, 1) double; % LOS azimuth angle
    end
    arguments (Output)
        value (:, 1) double
    end
    if ~isequal(size(t), size(a))
        error("t and a must be the same size");
    end

    n_t = 4;   % tilt beamwidth (higher = narrower beam)
    n_a = 4;   % azimuth rolloff sharpness (higher = more directional)

    % Elevation: cardioid family, null at zenith (t=180°), peak at nadir (t=0°)
    % Azimuth: cardioid family, peak at a=0° (+y), null at a=180° (-y)
    value = 10 .* n_t .* log10((1 + cosd(t)) ./ 2) + ...
            10 .* n_a .* log10((0.5 + 0.5 .* cosd(a)));
end
