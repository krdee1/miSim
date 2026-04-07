function [f, G] = plotGpsLogs(logDirs, seaToGroundLevel, plotWholeFlight)
    arguments (Input)
        logDirs (1, 1) string;
        seaToGroundLevel (1, 1) double = 110; % measured approximately from USGS national map viewer for the AERPAW test field
        plotWholeFlight (1, 1) logical = false;
    end
    arguments (Output)
        f (1, 1) matlab.ui.Figure;
        G cell;
    end
    % Plot setup
    f = uifigure;
    gf = geoglobe(f);
    hold(gf, "on");
    c = ["g", "b", "m", "c"]; % plotting colors
    
    % paths
    scenarioCsv = fullfile(matlab.project.rootProject().RootFolder, "aerpaw", "config", "scenario.csv");
    
    % configured data
    params = readScenarioCsv(scenarioCsv);
    
    fID = fopen(fullfile(matlab.project.rootProject().RootFolder, "aerpaw", "config", "client1.yaml"), 'r');
    yaml = fscanf(fID, '%s');
    fclose(fID);
    % origin (LLA)
    lla0 = [str2double(yaml((strfind(yaml, 'lat:') + 4):(strfind(yaml, 'lon:') - 1))), str2double(yaml((strfind(yaml, 'lon:') + 4):(strfind(yaml, 'alt:') - 1))), seaToGroundLevel];
    
    logDirs = dir(logDirs);
    logDirs = logDirs(3:end);
    logDirs = logDirs([logDirs.isdir] == 1);
    logDirs = logDirs(~startsWith({logDirs.name}, "controller_"));

    G = cell(size(logDirs));
    for ii = 1:size(logDirs, 1)
        % Find GPS log CSV
        gpsCsv = dir(fullfile(logDirs(ii).folder, logDirs(ii).name));
        gpsCsv = gpsCsv(endsWith({gpsCsv(:).name}, "_gps_log.csv"));
        gpsCsv = fullfile(gpsCsv.folder, gpsCsv.name);
    
        % Read GPS log CSV
        G{ii} = readGpsLogs(gpsCsv);
    
        % Find when algorithm begins/ends (using ENU altitude rate change)
        verticalSpeed = movmean(abs(diff(G{ii}.Altitude)), [10, 0]);
    
        % Automatically detect start/stop of algorithm flight (ignore takeoff, setup, return to liftoff, landing segments of flight)
        pctThreshold = 60; % pctThreshold may need adjusting depending on your flight
        startIdx = find(verticalSpeed <= prctile(verticalSpeed, pctThreshold), 1, "first");
        stopIdx = find(verticalSpeed <= prctile(verticalSpeed, pctThreshold), 1, "last");
    
        % % Plot whole flight, including setup/cleanup
        if plotWholeFlight
            startIdx = 1;
            stopIdx = length(verticalSpeed);
        end

        % Convert LLA trajectory data to ENU for external analysis
        % NaN out entries outside the algorithm flight range so they don't plot
        enu = NaN(height(G{ii}), 3);
        enu(startIdx:stopIdx, :) = lla2enu([G{ii}.Latitude(startIdx:stopIdx), G{ii}.Longitude(startIdx:stopIdx), G{ii}.Altitude(startIdx:stopIdx)], lla0, "flat");
        enu = array2table(enu, 'VariableNames', ["East", "North", "Up"]);
        G{ii} = [G{ii}, enu];

        % Do crude comparison of pairwise distances between this UAV and
        % all previous UAVs
        for jj = 1:(ii - 1)
            Ai = G{ii}(:, [1, end-2:end]);
            Aj = G{jj}(:, [1, end-2:end]);

            % Trim data to match sizes
            idx = min([size(Ai, 1), size(Aj, 1)]);
            Ai = Ai(1:idx, :); Aj = Aj(1:idx, :);

            pos_i = [Ai.East, Ai.North, Ai.Up];
            pos_j = [Aj.East, Aj.North, Aj.Up];
            d = vecnorm(pos_i - pos_j, 2, 2);
            d = d(~isnan(d));

            fprintf("Minimum distance between agents %d and %d is %2.3f\n", ii, jj, min(d));
            if min(d) < 6
                warning("Minimum distance between agents %d and %d of %2.3f is questionable for AERPAW", ii, jj, min(d));
            end
        end
    
        % Plot recorded trajectory over specified range of indices
        geoplot3(gf, G{ii}.Latitude(startIdx:stopIdx), G{ii}.Longitude(startIdx:stopIdx), G{ii}.Altitude(startIdx:stopIdx) + seaToGroundLevel, c(mod(ii, length(c))), 'LineWidth', 2, "MarkerSize", 5);
    end
    
    % Plot domain
    altOffset = 1; % to avoid clipping into the ground when displayed
    domain = [lla0; enu2lla(params.domainMax, lla0, "flat")];
    geoplot3(gf, [domain(1, 1), domain(2, 1), domain(2, 1), domain(1, 1), domain(1, 1)], [domain(1, 2), domain(1, 2), domain(2, 2), domain(2, 2), domain(1, 2)], repmat(domain(1, 3) + altOffset, 1, 5), 'LineWidth', 3, 'Color', 'k');
    geoplot3(gf, [domain(1, 1), domain(2, 1), domain(2, 1), domain(1, 1), domain(1, 1)], [domain(1, 2), domain(1, 2), domain(2, 2), domain(2, 2), domain(1, 2)], repmat(domain(2, 3) + altOffset, 1, 5), 'LineWidth', 3, 'Color', 'k');
    geoplot3(gf, [domain(1, 1), domain(1, 1)], [domain(1, 2), domain(1, 2)], domain(:, 3) + altOffset, 'LineWidth', 3, 'Color', 'k');
    geoplot3(gf, [domain(2, 1), domain(2, 1)], [domain(1, 2), domain(1, 2)], domain(:, 3) + altOffset, 'LineWidth', 3, 'Color', 'k');
    geoplot3(gf, [domain(1, 1), domain(1, 1)], [domain(2, 2), domain(2, 2)], domain(:, 3) + altOffset, 'LineWidth', 3, 'Color', 'k');
    geoplot3(gf, [domain(2, 1), domain(2, 1)], [domain(2, 2), domain(2, 2)], domain(:, 3) + altOffset, 'LineWidth', 3, 'Color', 'k');
    
    % Plot floor (minimum altitude constraint)
    floorAlt = params.minAlt;
    geoplot3(gf, [domain(1, 1), domain(2, 1), domain(2, 1), domain(1, 1), domain(1, 1)], [domain(1, 2), domain(1, 2), domain(2, 2), domain(2, 2), domain(1, 2)], repmat(domain(1, 3) + altOffset + floorAlt, 1, 5), 'LineWidth', 3, 'Color', 'r');
    
    % Plot objective
    objectivePos = [params.objectivePos, 0];
    llaObj = enu2lla(objectivePos, lla0, "flat");
    geoplot3(gf, [llaObj(1), llaObj(1)], [llaObj(2), llaObj(2)], [llaObj(3), domain(2, 3)], 'LineWidth', 3, "Color", 'y');
    
    % Plot obstacles
    for ii = 1:params.numObstacles
        obstacle = enu2lla([params.obstacleMin((1 + (ii - 1) * 3):(ii * 3)); params.obstacleMax((1 + (ii - 1) * 3):(ii * 3))], lla0, "flat");
        geoplot3(gf, [obstacle(1, 1), obstacle(2, 1), obstacle(2, 1), obstacle(1, 1), obstacle(1, 1)], [obstacle(1, 2), obstacle(1, 2), obstacle(2, 2), obstacle(2, 2), obstacle(1, 2)], repmat(obstacle(1, 3) + altOffset, 1, 5), 'LineWidth', 3, 'Color', 'r');
        geoplot3(gf, [obstacle(1, 1), obstacle(2, 1), obstacle(2, 1), obstacle(1, 1), obstacle(1, 1)], [obstacle(1, 2), obstacle(1, 2), obstacle(2, 2), obstacle(2, 2), obstacle(1, 2)], repmat(obstacle(2, 3) + altOffset, 1, 5), 'LineWidth', 3, 'Color', 'r');
        geoplot3(gf, [obstacle(1, 1), obstacle(1, 1)], [obstacle(1, 2), obstacle(1, 2)], obstacle(:, 3) + altOffset, 'LineWidth', 3, 'Color', 'r');
        geoplot3(gf, [obstacle(2, 1), obstacle(2, 1)], [obstacle(1, 2), obstacle(1, 2)], obstacle(:, 3) + altOffset, 'LineWidth', 3, 'Color', 'r');
        geoplot3(gf, [obstacle(1, 1), obstacle(1, 1)], [obstacle(2, 2), obstacle(2, 2)], obstacle(:, 3) + altOffset, 'LineWidth', 3, 'Color', 'r');
        geoplot3(gf, [obstacle(2, 1), obstacle(2, 1)], [obstacle(2, 2), obstacle(2, 2)], obstacle(:, 3) + altOffset, 'LineWidth', 3, 'Color', 'r');
    end
    
    % finish
    hold(gf, "off");
end