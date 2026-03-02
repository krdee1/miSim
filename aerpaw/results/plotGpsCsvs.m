% Plot setup
f = uifigure;
gf = geoglobe(f);
hold(gf, "on");
c = ["r", "g", "b", "m", "c", "y", "k"]; % plotting colors
seaToGroundLevel = 110; % meters, measured approximately from USGS national map viewer
lla0 = [35.72550610629396, -78.70019657805574, seaToGroundLevel]; % origin (LLA)

% Paths to logs
gpsCsvs = fullfile ("sandbox", "test4", ...
                   ["GPS_DATA_0c8d904aa159_2026-03-02_14:42:51.csv"; ...
                    "GPS_DATA_8e4f52dac04d_2026-03-02_14:42:52.csv"; ...
                   ]);

G = cell(size(gpsCsvs));
for ii = 1:size(gpsCsvs, 1)
     % Read CSV
    G{ii} = readGpsCsv(gpsCsvs(ii));

    % Find when algorithm begins/ends (using ENU altitude)
    enuTraj = lla2enu([G{ii}.Latitude, G{ii}.Longitude, G{ii}.Altitude], lla0, 'flat');

    verticalSpeed = movmean(abs(diff(G{ii}.Altitude)), [10, 0]);

    startIdx = find(verticalSpeed <= prctile(verticalSpeed, 25), 1, 'first');
    stopIdx = find(verticalSpeed <= prctile(verticalSpeed, 10), 1, 'last');
    % startIdx = 1;
    % stopIdx = length(verticalSpeed);

    speed = vecnorm(diff(enuTraj), 2, 2);
    meanSpeed = movmean(speed, [10, 0]);
    
    % Plot recorded trajectory
    geoplot3(gf, G{ii}.Latitude(startIdx:stopIdx), G{ii}.Longitude(startIdx:stopIdx), G{ii}.Altitude(startIdx:stopIdx) + seaToGroundLevel, c(mod(ii, length(c))), 'LineWidth', 2, "MarkerSize", 5);

end

% Plot objective
objectivePos = [35, 35, 0];
llaObj = enu2lla(objectivePos, lla0, 'flat');
geoplot3(gf, [llaObj(1), llaObj(1)], [llaObj(2), llaObj(2)], [llaObj(3), llaObj(3) + 50], 'LineWidth', 3);

% Plot domain
domain = [lla0; enu2lla([50, 50, 100], lla0, 'flat')];
geoplot3(gf, [domain(1, 1), domain(2, 1), domain(2, 1), domain(1, 1), domain(1, 1)], [domain(1, 2), domain(1, 2), domain(2, 2), domain(2, 2), domain(1, 2)], repmat(domain(1, 3) + 10, 1, 5), 'LineWidth', 3);

hold(gf, "off");