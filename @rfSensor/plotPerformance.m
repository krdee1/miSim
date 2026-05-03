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

    % Clear local caches so this visualization always uses its own grid
    obj.rssCache = [];
    for ii = 1:numel(otherSensors)
        otherSensors{ii}.rssCache = [];
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
    SNR  = 10.^(SNR/10);  SNR  = SNR  ./ max(SNR(:));  SNR  = 10 * log10(SNR);

    % Collect sensor positions and boresight parameters for overlay
    sensorXY      = [0, 0; otherSensorsPos(:, 1:2)];
    sensorTilts   = [obj.tilt;    cellfun(@(s) s.tilt,    otherSensors)];
    sensorAzimuths = [obj.azimuth; cellfun(@(s) s.azimuth, otherSensors)];
    tailScale = 0.5 * d;

    f = figure;
    tiledlayout(1, 2, TileSpacing="compact", Padding="compact");

    nexttile;
    imagesc(distances, distances, SNR);
    axis("image"); set(gca, 'YDir', 'normal');
    colorbar; xlabel("X (m)"); ylabel("Y (m)");
    title("Linearly Normalized SNR (dB)");
    subtitle("No interfering sources");
    addSensorOverlay(gca, sensorXY, sensorTilts, sensorAzimuths, tailScale);

    nexttile;
    imagesc(distances, distances, SINR);
    axis("image"); set(gca, 'YDir', 'normal');
    colorbar; xlabel("X (m)"); ylabel("Y (m)");
    title("Linearly Normalized SINR (dB)");
    subtitle(sprintf("%d interfering source(s)", size(otherSensorsPos, 1)));
    addSensorOverlay(gca, sensorXY, sensorTilts, sensorAzimuths, tailScale);
end

function addSensorOverlay(ax, sensorXY, tilts, azimuths, tailScale)
    % Draw a marker + boresight arrow for each sensor.
    % Tail direction follows azimuth convention (0=+Y, 90=+X, clockwise).
    % Tail length = tailScale * sind(tilt), so nadir (0°) has no tail and
    % horizon (90°) has the full tailScale length.
    hold(ax, 'on');
    for ii = 1:size(sensorXY, 1)
        x = sensorXY(ii, 1);
        y = sensorXY(ii, 2);
        if ii == 1
            c = [0, 0, 0];
            mk = 'o';
        else
            c = [0.9, 0.2, 0.2];
            mk = 'x';
        end
        scatter(ax, x, y, 80, c, mk, LineWidth=2);
        if tilts(ii) > 0
            u = tailScale * sind(tilts(ii)) * sind(azimuths(ii));
            v = tailScale * sind(tilts(ii)) * cosd(azimuths(ii));
            quiver(ax, x, y, u, v, 0, Color=c, LineWidth=2, MaxHeadSize=1.0);
        end
    end
    hold(ax, 'off');
end
