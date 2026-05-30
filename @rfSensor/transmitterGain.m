function value = transmitterGain(obj, theta)
    arguments (Input)
        obj   (1, 1) {mustBeA(obj, "rfSensor")};
        theta (:, 1) double; % angle from boresight (degrees)
    end
    arguments (Output)
        value (:, 1) double
    end
    % Cosine pattern: peak at boresight, -inf dB at 90°.
    % Clamped so targets behind the antenna (theta > 90°) get -inf rather than NaN.
    value = obj.constantGainTerm_dB + 10 .* obj.beamwidthExponent .* log10(max(0, cosd(theta)));
end
