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

    if isempty(obj.rssCache)
        [d, dx, dy, dz] = obj.computePointToPoints(agentPos, targetPos);
        obj.rssCache = 1e-3 .* 10 .^ (0.1 .* obj.RSS(d, dx, dy, dz));  % dBm → W
    end
    S = obj.rssCache;

    I = zeros(size(S));
    for ii = 1:size(otherSensors, 1)
        if isempty(otherSensors{ii}.rssCache)
            [d_o, dx_o, dy_o, dz_o] = otherSensors{ii}.computePointToPoints(otherSensorsPos(ii, 1:3), targetPos);
            otherSensors{ii}.rssCache = 1e-3 .* 10 .^ (0.1 .* otherSensors{ii}.RSS(d_o, dx_o, dy_o, dz_o));  % dBm → W
        end
        I = I + otherSensors{ii}.rssCache;
    end

    % Ground points more than 90 deg off boresight (reachable once pointing
    % is optimized) get exactly zero gain from the cos^n pattern, so S = 0
    % and the SINR would be -Inf dB. -Inf poisons the coverage sum and its
    % finite-difference gradient (NaN tilt/position) and ties every agent
    % with a -Inf partition threshold. Clamp to the smallest positive double:
    % those points stay hugely penalized but finite, and nothing changes
    % where S > 0.
    S = max(S, realmin);

    SINR = 10*log10(S ./ (I + obj.N));
    SNR  = 10*log10(S ./ obj.N);
end
