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

    % Temporary logic to make nadir-pointing most effective
    value = 10 .* log10(cosd(t) .^ 2) + 10 .* log10((0.5 + 0.5 .* cosd(a)) .^ 4);
end
