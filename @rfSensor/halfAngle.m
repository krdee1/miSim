function value = halfAngle(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
    end
    arguments (Output)
        value (1, 1) double;
    end
    dtheta = (0:0.1:179.9)';
    gain = obj.transmitterGain(dtheta);
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
