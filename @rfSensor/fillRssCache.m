function obj = fillRssCache(obj, agentPos, targetPos)
    % Received power (W) from a transmitter at agentPos at every target, in
    % rssCache. Used by sensorPerformance and for the full-grid coverage in
    % agent.run.
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        agentPos (1, 3) double;
        targetPos (:, 3) double;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
    end

    [d, dx, dy, dz] = obj.computePointToPoints(agentPos, targetPos);
    obj.rssCache = 1e-3 .* 10 .^ (0.1 .* obj.RSS(d, dx, dy, dz));  % dBm -> W
end
