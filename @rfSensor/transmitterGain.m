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

    n = 4;   % beamwidth exponent (higher = narrower beam)

    % Angular offset from boresight via spherical law of cosines
    % Convention: t=0° nadir, t=90° horizon; a=0° +y, a=90° +x
    cos_theta = sind(obj.boresightTilt) .* sind(t) .* cosd(a - obj.boresightAzimuth) + ...
                cosd(obj.boresightTilt) .* cosd(t);
    cos_theta = max(-1, min(1, cos_theta));  % clamp for numerical safety
    theta = acosd(cos_theta);

    % Cardioid family: peak at boresight (theta=0), null opposite (theta=180°)
    value = 10 .* n .* log10((1 + cosd(theta)) ./ 2);
end
