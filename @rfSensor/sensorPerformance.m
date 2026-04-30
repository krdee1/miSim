function [SINR, SNR, obj, otherSensors] = sensorPerformance(obj, agentPos, targetPos, otherSensorsPos, otherSensors)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        agentPos (1, 3) double;
        targetPos (:, 3) double;
        otherSensorsPos (:, 3) double = [];
        otherSensors (:, 1) cell = {};
    end
    arguments (Output)
        SINR (:, 1) double;
        SNR (:, 1) double;
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        otherSensors (:, 1) cell;
    end
    assert(size(otherSensorsPos, 1) == size(otherSensors, 1), "Mismatch in number of other sensor positions (%d) and number of other sensors (%d) provided", size(otherSensorsPos, 1), size(otherSensors, 1));

    [d, t, a] = obj.computePointToPoints(agentPos, targetPos);

    if isempty(obj.rssCache)
        obj.rssCache = 10 .^ (0.1 .* obj.RSS(d, t, a));
    end
    S = obj.rssCache;

    I = zeros(size(d));
    for ii = 1:size(otherSensors, 1)
        if isempty(otherSensors{ii}.rssCache)
            [d_other, t_other, a_other] = otherSensors{ii}.computePointToPoints(otherSensorsPos(ii, 1:3), targetPos);
            otherSensors{ii}.rssCache = 10 .^ (0.1 .* otherSensors{ii}.RSS(d_other, t_other, a_other));
        end
        I = I + otherSensors{ii}.rssCache;
    end

    SINR = 10*log10(S ./ (I + obj.N));
    SNR = 10*log10(S ./ obj.N);
end
