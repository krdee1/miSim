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

    otherSensorsPos = otherSensorsPos + [0, 0, altitude];

    % Create grid on which to evalute SINR, SNR
    agentPos = [0, 0, altitude];
    d = 10;
    if ~isempty(otherSensorsPos)
        d = max(d, max(vecnorm(otherSensorsPos(:, 1:2), 2, 2)) * 1.25);
    end
    c = 0.1;
    d = ceil(d / c) * c;
    distances = -d:c:d;
    [targetPosX, targetPosY] = meshgrid(distances, distances);

    % Compute SINR, SNR
    [SINR, SNR] = obj.sensorPerformance(agentPos, [targetPosX(:), targetPosY(:), zeros(size(targetPosX(:)))], otherSensorsPos, otherSensors);
    SINR = reshape(SINR, size(targetPosX));
    SNR = reshape(SNR, size(targetPosX));

    % normalize in linear scale
    SINR = 10.^(SINR/10); SINR = SINR ./ max(SINR(:)); SINR = 10 * log10(SINR);
    SNR  = 10.^(SNR/10);  SNR  = SNR  ./ max(SNR(:)); SNR = 10 * log10(SNR);
    
    f = figure;
    tiledlayout(1, 2, TileSpacing="compact", Padding="compact");

    nexttile;
    imagesc(distances, distances, SNR);
    axis("image"); set(gca, 'YDir', 'normal');
    colorbar;
    xlabel("X (m)"); ylabel("Y (m)");
    title("Linearly Normalized SNR (dB)");
    subtitle("No interfering sources");

    nexttile;
    imagesc(distances, distances, SINR);
    axis("image"); set(gca, 'YDir', 'normal');
    colorbar;
    xlabel("X (m)"); ylabel("Y (m)");
    title("Linearly Normalized SINR (dB)");
    subtitle(sprintf("%d interfering source(s)", size(otherSensorsPos, 1)));
end