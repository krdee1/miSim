% Plot setup
f = uifigure;
gf = geoglobe(f);
hold(gf, "on");
c = ["r", "g", "b", "m", "c", "y", "k"]; % plotting colors
seaToGroundLevel = 110; % meters, measured approximately from USGS national map viewer

% Paths to logs
gpsCsvs = fullfile ("sandbox", "test2", ...
                   ["GPS_DATA_0c8d904aa159_2026-02-27_20:52:33.csv"; ...
                    "GPS_DATA_8e4f52dac04d_2026-02-27_20:52:33.csv"; ...
                   ]);

G = cell(size(gpsCsvs));
for ii = 1:size(gpsCsvs, 1)
     % Read CSV
    G{ii} = readGpsCsv(gpsCsvs(ii));

    % Plot recorded trajectory
    geoplot3(gf, G{ii}.Latitude, G{ii}.Longitude, G{ii}.Altitude + seaToGroundLevel, c(mod(ii, length(c))), 'LineWidth', 2, "MarkerSize", 5);

end
hold(gf, "off");