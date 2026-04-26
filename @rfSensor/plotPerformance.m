function f = plotPerformance(obj, altitude, otherSensorsPos, otherSensors)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
        altitude (1, 1) double;
        otherSensorsPos (:, 3) double = NaN(0, 3);
        otherSensors (:, 1) cell = cell(0, 1);
    end
    arguments (Output)
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")};
    end

    % Create grid on which to evalute SINR, SNR
    agentPos = [0, 0, altitude];
    d = max(10, max(vecnorm(otherSensorsPos(1:2), 2, 2)) * 1.25);
    c = 0.1;
    d = ceil(d / c) * c;
    distances = -d:c:d;
    [targetPosX, targetPosY] = meshgrid(distances, distances);

    % Compute SINR, SNR
    [SINR, SNR] = obj.sensorPerformance(agentPos, [targetPosX(:), targetPosY(:), zeros(size(targetPosX(:)))], otherSensorsPos, otherSensors);
    SINR = reshape(SINR, size(targetPosX));
    SNR = reshape(SNR, size(targetPosX));

    % normalize
    SINR = SINR ./ max(SINR);
    SNR = SNR ./ max(SNR);
    
    f = figure;
    imagesc(SNR);
    axis("image");
    colorbar;
    title("Normalized SNR");

    f = figure;
    imagesc(SINR);
    axis("image");
    colorbar;
    title("Normalized SINR");
    
end