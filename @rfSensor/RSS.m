function value = RSS(obj, d, t, a)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        d (:, 1) double; % distance from agent to target
        t (:, 1) double; % LOS tilt angle
        a (:, 1) double; % LOS azimuth angle
    end
    arguments (Output)
        value (:, 1) double
    end
    assert(size(d, 1) == size(t, 1), "Mismatch in number of distances (%d) and tilts (%d) provided", size(d, 1), size(t, 1));

    % RSS (dBm) = TX Power (dBm) + TX Antenna Gain (dBi) + RX Antenna Gain (dBi) - Path Loss (dB)
    value = obj.P_TX_dBm + obj.transmitterGain(t, a) + obj.G_RX_dBi - obj.pathLoss(d);
end