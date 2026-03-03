% Plot setup
f = uifigure;
gf = geoglobe(f);
hold(gf, "on");
c = ["g", "b", "m", "c"]; % plotting colors

% paths
scenarioCsv = fullfile("aerpaw", "config", "scenario.csv");

% configured data
params = readScenarioCsv(scenarioCsv);

% coordinate system constants
seaToGroundLevel = 110; % meters, measured approximately from USGS national map viewer

fID = fopen(fullfile("aerpaw", "config", "client.yaml"), 'r');
yaml = fscanf(fID, '%s');
fclose(fID);
% origin (LLA)
lla0 = [str2double(yaml((strfind(yaml, 'lat:') + 4):(strfind(yaml, 'lon:') - 1))), str2double(yaml((strfind(yaml, 'lon:') + 4):(strfind(yaml, 'alt:') - 1))), seaToGroundLevel];

% Paths to logs
gpsCsvs = dir(fullfile("sandbox", "test6", "*.csv"));

G = cell(size(gpsCsvs));
for ii = 1:size(gpsCsvs, 1)
     % Read CSV
    G{ii} = readGpsCsv(fullfile(gpsCsvs(ii).folder, gpsCsvs(ii).name));

    % Find when algorithm begins/ends (using ENU altitude rate change)
    enuTraj = lla2enu([G{ii}.Latitude, G{ii}.Longitude, G{ii}.Altitude], lla0, 'flat');

    verticalSpeed = movmean(abs(diff(G{ii}.Altitude)), [10, 0]);

    % Automatically detect start/stop of algorithm flight (ignore takeoff, setup, return to liftoff, landing segments of flight)
    pctThreshold = 90; % pctThreshold may need adjusting depending on your flight
    startIdx = find(verticalSpeed <= prctile(verticalSpeed, pctThreshold), 1, 'first'); 
    stopIdx = find(verticalSpeed <= prctile(verticalSpeed, pctThreshold), 1, 'last');

    % % Plot whole flight, including setup/cleanup
    % startIdx = 1;
    % stopIdx = length(verticalSpeed);
    
    % Plot recorded trajectory over specified range of indices
    geoplot3(gf, G{ii}.Latitude(startIdx:stopIdx), G{ii}.Longitude(startIdx:stopIdx), G{ii}.Altitude(startIdx:stopIdx) + seaToGroundLevel, c(mod(ii, length(c))), 'LineWidth', 2, "MarkerSize", 5);
end

% Plot objective
objectivePos = [params.objectivePos, 0];
llaObj = enu2lla(objectivePos, lla0, 'flat');
geoplot3(gf, [llaObj(1), llaObj(1)], [llaObj(2), llaObj(2)], [llaObj(3), llaObj(3) + 50], 'LineWidth', 3, "Color", 'y');

% Plot domain
altOffset = 1; % to avoid clipping into the ground when displayed
domain = [lla0; enu2lla(params.domainMax, lla0, 'flat')];
geoplot3(gf, [domain(1, 1), domain(2, 1), domain(2, 1), domain(1, 1), domain(1, 1)], [domain(1, 2), domain(1, 2), domain(2, 2), domain(2, 2), domain(1, 2)], repmat(domain(1, 3) + altOffset, 1, 5), 'LineWidth', 3, 'Color', 'k');
geoplot3(gf, [domain(1, 1), domain(2, 1), domain(2, 1), domain(1, 1), domain(1, 1)], [domain(1, 2), domain(1, 2), domain(2, 2), domain(2, 2), domain(1, 2)], repmat(domain(2, 3) + altOffset, 1, 5), 'LineWidth', 3, 'Color', 'k');
geoplot3(gf, [domain(1, 1), domain(1, 1)], [domain(1, 2), domain(1, 2)], domain(:, 3) + altOffset, 'LineWidth', 3, 'Color', 'k');
geoplot3(gf, [domain(2, 1), domain(2, 1)], [domain(1, 2), domain(1, 2)], domain(:, 3) + altOffset, 'LineWidth', 3, 'Color', 'k');
geoplot3(gf, [domain(1, 1), domain(1, 1)], [domain(2, 2), domain(2, 2)], domain(:, 3) + altOffset, 'LineWidth', 3, 'Color', 'k');
geoplot3(gf, [domain(2, 1), domain(2, 1)], [domain(2, 2), domain(2, 2)], domain(:, 3) + altOffset, 'LineWidth', 3, 'Color', 'k');

% Plot floor (minimum altitude constraint)
floorAlt = params.minAlt;
geoplot3(gf, [domain(1, 1), domain(2, 1), domain(2, 1), domain(1, 1), domain(1, 1)], [domain(1, 2), domain(1, 2), domain(2, 2), domain(2, 2), domain(1, 2)], repmat(domain(1, 3) + altOffset + floorAlt, 1, 5), 'LineWidth', 3, 'Color', 'r');

% finish
hold(gf, "off");