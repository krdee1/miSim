function value = RSS(obj, d, t)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        d (:, 1) double; % distance from agent to target
        t (:, 1) double; % LOS tilt angle
    end
    arguments (Output)
        value (:, 1) double
    end
    
    rho_dBm = 10*log10(obj.P_TX/1e-3);

    % RSS = TX Power + Antenna Gain - Path Loss
    value = rho_dBm + obj.antennaGain(t) - obj.pathLoss(d);

end