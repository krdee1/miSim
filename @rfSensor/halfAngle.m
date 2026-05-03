function value = halfAngle(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
    end
    arguments (Output)
        value (1, 1) double;
    end
    % Sweep angular offset from boresight by evaluating transmitterGain at
    % (obj.tilt + dtheta, obj.azimuth). The cosine difference identity guarantees
    % the resulting angular offset from boresight equals dtheta exactly,
    % independent of the actual pointing direction.
    dtheta = (0:0.1:179.9)';
    gain = obj.transmitterGain(obj.tilt + dtheta, obj.azimuth * ones(size(dtheta)));
    target = gain(1) - 3;
    idx = find(gain <= target, 1);
    if isempty(idx) || idx == 1
        value = dtheta(end);
        return;
    end
    % Linear interpolation between bracketing samples
    value = dtheta(idx-1) + (target - gain(idx-1)) * ...
            (dtheta(idx) - dtheta(idx-1)) / (gain(idx) - gain(idx-1));
end
