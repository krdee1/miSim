function value = antennaGain(obj, t)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        t (:, 1) double; % LOS tilt angle
    end
    arguments (Output)
        value (:, 1) double
    end

    %% TODO

    % Temporary logic to make nadir-pointing most effective
    value = 10*log10(cosd(t) .^ 8);

    % % Temporary logic for 0 dB at all tilt angles
    % value = zeros(size(t));
end
