function value = RSS(obj, d, dx, dy, dz)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        d  (:, 1) double;
        dx (:, 1) double;
        dy (:, 1) double;
        dz (:, 1) double;
    end
    arguments (Output)
        value (:, 1) double
    end
    % Boresight unit vector: [st*sa, st*ca, -ct]
    % Target direction unit vector: [dx, dy, dz] / d
    % cos_theta = dot product of the two, computed without per-point trig.
    st = sind(obj.tilt);
    ct = cosd(obj.tilt);
    sa = sind(obj.azimuth);
    ca = cosd(obj.azimuth);
    cos_theta = (st .* (dx .* sa + dy .* ca) - ct .* dz) ./ max(d, eps);
    cos_theta = max(-1, min(1, cos_theta));
    theta = acosd(cos_theta);
    gain  = 10 .* obj.beamwidthExponent .* log10((1 + cosd(theta)) ./ 2);
    value = obj.P_TX_dBm + gain + obj.G_RX_dBi - obj.pathLoss(d);
end
