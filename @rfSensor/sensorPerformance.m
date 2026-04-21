function SINR = sensorPerformance(obj, d, t, d_other, t_other, otherSensors)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        d (:, 1) double;
        t(:, 1) double;
        d_other (:, :) double = [];
        t_other (:, :) double = [];
        otherSensors (1, :) cell = {};
    end
    arguments (Output)
        SINR (:, 1) double;
    end
    assert(size(d, 1) == size(t, 1), "Mismatch in number of distance (%d) and angle (%d) pairs provided", size(d, 1), size(t, 1));
    assert(size(d_other, 1) == size(t_other, 1), "Mismatch in number of distances (%d) and tilts (%d) provided to other sensors", size(t, 1), size(t_other, 1));
    assert(size(d_other, 2) == size(t_other, 2), "Mismatch in number of other sensors given distances (%d) and tilts (%d)", size(d_other, 1), size(t_other, 1));
    assert(size(otherSensors, 2) == size(d_other, 2), "Mismatch in number of distances from other sensors (%d) and number of other sensors (%d) provided", size(d_other, 2), size(otherSensors, 2));

    % Performance is measured as SINR for this sensor
    S = 10 .^ (0.1 * obj.RSS(d, t)); % Signal
    I = zeros(size(d)); % Interference from other agents
    for ii = 1:size(otherSensors, 2)
        I = I + 10 .^ (0.1 * otherSensors{ii}.RSS(d_other(:, ii), t_other(:, ii)));
    end

    SINR = 10*log10(S ./ (I + obj.N));
end