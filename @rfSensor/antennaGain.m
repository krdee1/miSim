function value = antennaGain(obj, t)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        t (:, 1) double; % LOS tilt angle
    end
    arguments (Output)
        value (:, 1) double
    end

    % TODO
    value = 10*log10(1);
end
