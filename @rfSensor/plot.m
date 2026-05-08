function f = plot(obj, altitude, otherSensorsPos, otherSensors)
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

    % bias other sensors altitudes appropriately
    otherSensorsPos = otherSensorsPos + [0, 0, altitude];

    % Create grid on which to evalute SINR, SNR
    agentPos = [0, 0, altitude];
    d = 10;
    if ~isempty(otherSensorsPos)
        d = max(otherSensorsPos(:, 3) * 0.55);
        d = max(d, max(vecnorm(otherSensorsPos(:, 1:2), 2, 2)) * 1.25);
    end
    c = 0.1;
    d = ceil(d / c) * c;
    distances = -d:c:d;
    [targetPosX, targetPosY] = meshgrid(distances, distances);

    % Compute SINR, SNR
    [SINR, ~] = obj.sensorPerformance(agentPos, [targetPosX(:), targetPosY(:), zeros(size(targetPosX(:)))], otherSensorsPos, otherSensors);
    SINR = reshape(SINR, size(targetPosX));

    % normalize in linear scale
    % SINR = 10.^(SINR/10); SINR = SINR ./ max(SINR(:)); SINR = 10 * log10(SINR);

    % Collect sensor positions and boresight parameters for overlay
    sensorTilts   = [obj.tilt;    cellfun(@(s) s.tilt,    otherSensors)];
    sensorAzimuths = [obj.azimuth; cellfun(@(s) s.azimuth, otherSensors)];
    tailScale = 0.5 * d;

    f = figure;
    surf(targetPosX, targetPosY, zeros(size(targetPosX)), SINR, "EdgeColor", "none");
    axis(f.Children(1), "image");
    colormap(f.Children(1), "hot");
    title("Ground User SINR and -3 dB antenna gain regions");
    subtitle(sprintf("%d interfering source(s)", size(otherSensorsPos, 1)));
    c = colorbar;
    ylabel(c, "SINR (dB)");
    xlabel("X (m)");
    ylabel("Y (m)");
    hold(f.Children(2), "on");
    scatter3(0, 0, altitude, 100, 'ko', "LineWidth", 2);
    scatter3(otherSensorsPos(:, 1), otherSensorsPos(:, 2), otherSensorsPos(:, 3), 100, "bx", "LineWidth", 2);
    qSelf = quiver3(0, 0, altitude, ...
        tailScale * sind(obj.tilt) * sind(obj.azimuth), ...
        tailScale * sind(obj.tilt) * cosd(obj.azimuth), ...
        -tailScale * cosd(obj.tilt), ...
        0, 'k', 'LineWidth', 1.5);
    qSelf.MaxHeadSize = 0.75;
    if ~isempty(otherSensors)
        qOthers = quiver3(otherSensorsPos(:,1), otherSensorsPos(:,2), otherSensorsPos(:,3), ...
            tailScale .* sind(sensorTilts(2:end)) .* sind(sensorAzimuths(2:end)), ...
            tailScale .* sind(sensorTilts(2:end)) .* cosd(sensorAzimuths(2:end)), ...
            -tailScale .* cosd(sensorTilts(2:end)), ...
            0, 'b', 'LineWidth', 1.5);
        qOthers.MaxHeadSize = 0.75;
    end
    % Draw half-angle cones co-boresighted with each quiver arrow
    N = 48;
    phi = linspace(0, 2*pi, N);
    [PHI, S] = meshgrid(phi, [0; 1]);   % row 1 = apex (s=0), row 2 = base (s=1)
    allSensors = [{obj}; otherSensors];
    allPos     = [[0, 0, altitude]; otherSensorsPos];
    for ii = 1:numel(allSensors)
        ha  = allSensors{ii}.halfAngle();
        tlt = sensorTilts(ii);
        az  = sensorAzimuths(ii);
        pos = allPos(ii, :);
        % Cone length: enough that the axis tip is guaranteed below z=0
        coneLength = 1.1 * pos(3) / max(cosd(tlt), 0.1);
        % Nadir cone mesh: apex at origin, base at z = -coneLength
        cX = S .* coneLength .* tand(ha) .* cos(PHI);
        cY = S .* coneLength .* tand(ha) .* sin(PHI);
        cZ = -S .* coneLength;
        % Rotate nadir → boresight (same convention as quiver arrows)
        Ry = [cosd(tlt), 0, -sind(tlt); 0, 1, 0; sind(tlt), 0, cosd(tlt)];
        Rz = [sind(az), -cosd(az), 0; cosd(az), sind(az), 0; 0, 0, 1];
        R  = Rz * Ry;
        pts = R * [cX(:)'; cY(:)'; cZ(:)'];
        cX  = reshape(pts(1,:), size(cX)) + pos(1);
        cY  = reshape(pts(2,:), size(cY)) + pos(2);
        cZ  = reshape(pts(3,:), size(cZ)) + pos(3);
        if ii == 1
            fc = [0, 0, 0];
        else
            fc = [0, 0, 1];
        end
        surf(cX, cY, cZ, "FaceColor", fc, "FaceAlpha", 0.15, "EdgeColor", "none");

        % Conic section: intersect each cone generator with z=0
        b_vec = R * [0; 0; -1];
        u_vec = R * [1; 0; 0];
        v_vec = R * [0; 1; 0];
        phi_sec = linspace(0, 2*pi, 720)';
        dirs = cosd(ha) .* b_vec' + sind(ha) .* (cos(phi_sec) .* u_vec' + sin(phi_sec) .* v_vec');
        t_sec = -pos(3) ./ dirs(:, 3);
        t_sec(t_sec <= 0) = NaN;
        sx = pos(1) + t_sec .* dirs(:, 1);
        sy = pos(2) + t_sec .* dirs(:, 2);
        plot3(sx, sy, zeros(size(sx)), "Color", fc, "LineWidth", 2);
    end
    clim(f.Children(2), [min(SINR(:)), max(SINR(:))]);
    xlim(f.Children(2), [-d, d]);
    ylim(f.Children(2), [-d, d]);
    hold(f.Children(2), "off");
    zlim([0, Inf]);
    



end