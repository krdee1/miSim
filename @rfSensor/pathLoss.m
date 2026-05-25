function L_FSPL_dB = pathLoss(obj, d)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        d (:, 1) double; % distance from TX to RX
    end
    arguments (Output)
        L_FSPL_dB (:, 1) double
    end

    % Free Space Path Loss (dB); d clamped away from zero (log undefined at d=0)
    L_FSPL_dB = obj.lossExponent * 10 * log10(max(d, eps)) + 20 * log10(obj.f_c) + 20 * log10((4*pi)/obj.c);

end
