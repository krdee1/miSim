function [f, corners] = plotScenarioGlobe(scenarios, opts)
% plotScenarioGlobe  Show AERPAW scenario geometry on a 3-D globe before
% flying it, e.g. to check the domain against the testbed geofence.
%
%   plotScenarioGlobe()                                    % the three service scenarios
%   plotScenarioGlobe("scenario_walls")                    % any aerpaw/config/<name>.csv
%   plotScenarioGlobe(SimFolder="sandbox/aerpawService_2026_11_18_10_48_37")
%
% Draws, on a geoglobe at the ENU origin of aerpaw/config/client1.yaml:
%   black   domain box (walls and ceiling)
%   red     minimum-altitude floor
%   colored obstacles, one color per scenario (shared domain drawn once)
%   circles start positions; with SimFolder, the simulated tracks
%           (<scenario>_miSimHist.mat, scenario name without "scenario_service_")
% and prints the latitude/longitude of the domain corners.
%
% Requires Mapping Toolbox (geoglobe, geoplot3) and enu2lla (UAV, Navigation
% or Sensor Fusion and Tracking Toolbox), as plotGpsLogs.m does.
arguments
    scenarios (1, :) string = ["scenario_service_L1U1", "scenario_service_L2U2", "scenario_service_L3U3"]
    opts.SimFolder (1, 1) string = ""
    opts.SeaToGroundLevel (1, 1) double = 110 % m, as in plotGpsLogs.m (USGS, AERPAW test field)
end
root = matlab.project.rootProject().RootFolder;

% ENU origin (LLA), same parsing as plotGpsLogs.m
fID = fopen(fullfile(root, "aerpaw", "config", "client1.yaml"), 'r');
yaml = fscanf(fID, '%s');
fclose(fID);
lla0 = [str2double(yaml((strfind(yaml, 'lat:') + 4):(strfind(yaml, 'lon:') - 1))), ...
        str2double(yaml((strfind(yaml, 'lon:') + 4):(strfind(yaml, 'alt:') - 1))), opts.SeaToGroundLevel];

f = uifigure("Name", "Scenario geometry");
gf = geoglobe(f);
hold(gf, "on");
colors = ["m", "c", "y", "g"];
uavColors = ["#D55E00", "#009E73", "#0072B2", "#CC79A7"];
altOffset = 1; % m, keeps ground-level lines from clipping into the terrain

drawnDomains = zeros(0, 6);
for ss = 1:numel(scenarios)
    p = readScenarioCsv(fullfile(root, "aerpaw", "config", scenarios(ss) + ".csv"));
    box = [p.domainMin, p.domainMax];
    if ~ismember(box, drawnDomains, "rows")
        drawnDomains(end + 1, :) = box; %#ok<AGROW>
        drawBox(gf, p.domainMin, p.domainMax, lla0, "k", altOffset);
        % Floor at minAlt
        ring = [p.domainMin(1), p.domainMin(2); p.domainMax(1), p.domainMin(2); ...
                p.domainMax(1), p.domainMax(2); p.domainMin(1), p.domainMax(2); p.domainMin(1), p.domainMin(2)];
        q = enu2lla([ring, repmat(p.minAlt, 5, 1)], lla0, "flat");
        geoplot3(gf, q(:, 1), q(:, 2), q(:, 3), "r", "LineWidth", 2);

        corners = enu2lla([ring(1:4, :), zeros(4, 1)], lla0, "flat");
        fprintf("Domain [%g %g %g] to [%g %g %g] m ENU (ceiling %g m above ground). Corners:\n", box, p.domainMax(3));
        names = ["SW (xMin, yMin)", "SE (xMax, yMin)", "NE (xMax, yMax)", "NW (xMin, yMax)"];
        for kk = 1:4
            fprintf("  %-16s lat %.7f, lon %.7f\n", names(kk), corners(kk, 1), corners(kk, 2));
        end
    end

    % Obstacles
    obsMin = reshape(p.obstacleMin, 3, [])';
    obsMax = reshape(p.obstacleMax, 3, [])';
    for oo = 1:p.numObstacles
        drawBox(gf, obsMin(oo, :), obsMax(oo, :), lla0, colors(mod(ss - 1, numel(colors)) + 1), altOffset);
    end

    % Starts and simulated tracks
    starts = reshape(p.initialPositions, 3, [])';
    for ii = 1:size(starts, 1)
        q = enu2lla(starts(ii, :), lla0, "flat");
        geoplot3(gf, q(1), q(2), q(3), "o", "Color", uavColors(ii), "MarkerSize", 8, "LineWidth", 2);
    end
    if opts.SimFolder ~= ""
        histFile = fullfile(opts.SimFolder, erase(scenarios(ss), "scenario_service_") + "_miSimHist.mat");
        if isfile(histFile)
            h = load(histFile).out;
            for ii = 1:numel(h.agent)
                q = enu2lla(h.agent(ii).pos, lla0, "flat");
                geoplot3(gf, q(:, 1), q(:, 2), q(:, 3), "Color", uavColors(ii), "LineWidth", 2);
            end
        else
            warning("No simulated track %s", histFile);
        end
    end
end
hold(gf, "off");
end

function drawBox(gf, bMin, bMax, lla0, color, altOffset)
% Wireframe of an axis-aligned ENU box
x = [bMin(1), bMax(1)];
y = [bMin(2), bMax(2)];
z = [bMin(3) + altOffset, bMax(3)];
ring = [x(1), y(1); x(2), y(1); x(2), y(2); x(1), y(2); x(1), y(1)];
for zz = z
    q = enu2lla([ring, repmat(zz, 5, 1)], lla0, "flat");
    geoplot3(gf, q(:, 1), q(:, 2), q(:, 3), color, "LineWidth", 2);
end
for kk = 1:4
    q = enu2lla([ring(kk, :), z(1); ring(kk, :), z(2)], lla0, "flat");
    geoplot3(gf, q(:, 1), q(:, 2), q(:, 3), color, "LineWidth", 2);
end
end
