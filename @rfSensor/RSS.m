function value = RSS(obj, d, t)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        d (:, 1) double; % distance from agent to target
        t (:, 1) double; % LOS tilt angle
    end
    arguments (Output)
        value (:, 1) double
    end
    assert(size(d, 1) == size(t, 1), "Mismatch in number of distances (%d) and tilts (%d) provided", size(d, 1), size(t, 1));

    % RSS (dBm) = TX Power (dBm) + Antenna Gain (dB) - Path Loss (dB)
    value = obj.P_TX_dBm + obj.antennaGain(t) - obj.pathLoss(d);
end