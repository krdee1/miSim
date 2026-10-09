classdef test_thesis_plots < matlab.unittest.TestCase
    % test class that generates plots I use in my thesis
    methods (Test)
        function service_performance(tc)
            % TX parameters
            P_TX = 10e-3;
            T_0 = 300;
            BW = 20e6;
            f_c = 2e9;
            G_RX_dBi = 3;
            beamwidthExponent = [16, 12, 32];
            elevation = [30, 20, 45];
            azimuth = [15, 325, 200];
            lossExponent = 2;

            % TX Positions
            pos1 = [50,  50,  35];
            pos2 = [70, 30,  30];
            pos3 = [40, 75,  40];

            % User Distribution Parameters
            mu = [78, 66];
            sig = reshape([2000, 1100; 1100, 2000], [1, 2, 2]);

            sensor1 = rfSensor;
            sensor1 = sensor1.initialize(P_TX, T_0, BW, f_c, G_RX_dBi, beamwidthExponent(1), elevation(1), azimuth(1), lossExponent);
            sensor2 = rfSensor;
            sensor2 = sensor2.initialize(P_TX, T_0, BW, f_c, G_RX_dBi, beamwidthExponent(2), elevation(2), azimuth(2), lossExponent);
            sensor3 = rfSensor;
            sensor3 = sensor3.initialize(P_TX, T_0, BW, f_c, G_RX_dBi, beamwidthExponent(3), elevation(3), azimuth(3), lossExponent);

            % Compute sensor and system performance for a single user distribution
            domain = rectangularPrism;
            domain = domain.initialize([zeros(1, 3); 100 * ones(1, 3)], "DOMAIN", "Domain");

            objectiveFunction = objectiveFunctionWrapper(mu, sig);
            domain.objective = domain.objective.initialize(objectiveFunction, domain, 0.1, 0, 1e-6, mu, sig);

            % Build target grid matching the domain objective grid and flatten user distribution
            nX = size(domain.objective.values, 2);
            nY = size(domain.objective.values, 1);
            x_domain = linspace(0, 100, nX);
            y_domain = linspace(0, 100, nY);
            [Xd, Yd] = meshgrid(x_domain, y_domain);
            targetPos_domain = [Xd(:), Yd(:), zeros(numel(Xd), 1)];
            userDist = domain.objective.values(:) / sum(domain.objective.values, "all");

            [SINR1, ~, sensor1, others] = sensor1.sensorPerformance(pos1, targetPos_domain, [pos2; pos3], {sensor2; sensor3});
            sensor2 = others{1}; sensor3 = others{2};
            [SINR2, ~, sensor2, others] = sensor2.sensorPerformance(pos2, targetPos_domain, [pos1; pos3], {sensor1; sensor3});
            sensor1 = others{1}; sensor3 = others{2};
            [SINR3, ~, sensor3, ~]      = sensor3.sensorPerformance(pos3, targetPos_domain, [pos1; pos2], {sensor1; sensor2});

            maxSINR_linear = max([10.^(SINR1/10), 10.^(SINR2/10), 10.^(SINR3/10)], [], 2);

            % Plot SINR from each UAV's perspective with a shared color scale,
            % one top-down map per UAV, in the style of the sigmoid sensor figure
            sharedClim = [-80, max([SINR1; SINR2; SINR3])];
            SINR = {SINR1, SINR2, SINR3};
            sensors = {sensor1, sensor2, sensor3};
            positions = [pos1; pos2; pos3];
            for ii = 1:3
                f = plotSinrMap(x_domain, y_domain, reshape(SINR{ii}, size(Xd)), ...
                    sensors, positions, ii, sharedClim);
                if ii == 3 % only plot for #3 right now
                    f = exportThesisFig(f, "sinrexample" + ii, 0.95, Colormap="parula", Recolor=false, Resolution=300);
                end
                close(f);
            end
        end
        function sigmoid_sensor_performance(tc)
            s = sigmoidSensor;
            s = s.initialize(65, 1, 50, 0.2, 30, 120);
            p = 50 * ones(1, 3);
            mu = [50, 50];
            sig = 25 * reshape(eye(2), [1, 2, 2]);
            domain = rectangularPrism;
            domain = domain.initialize([zeros(1, 3); 100 * ones(1, 3)], "DOMAIN", "Domain");
            objectiveFunction = objectiveFunctionWrapper(mu, sig);
            domain.objective = domain.objective.initialize(objectiveFunction, domain, 0.1, 0, 1e-6, mu, sig);

            S = s.sensorPerformance(p, [domain.objective.X(:), domain.objective.Y(:), zeros(size(domain.objective.X(:)))]);
            S = reshape(S, size(domain.objective.X));

            % meshgrid puts y down the rows, so plot against the grid vectors
            % and put y = 0 at the bottom (imagesc defaults to row 1 at the top)
            x = domain.objective.X(1, :);
            y = domain.objective.Y(:, 1);

            f = figure;
            imagesc(x, y, S);
            set(gca, "YDir", "normal");
            axis("image");
            hold("on");
            % Boresight direction: the arrow runs from the UAV to where the
            % boresight meets the ground, so its length grows with tilt
            % (altitude * tan(tilt)). Azimuth 0 = +Y, 90 = +X, as in sensorPerformance.
            reach = p(3) * tand(s.tilt);
            hit = p(1:2) + reach * [sind(s.azimuth), cosd(s.azimuth)];
            quiver(p(1), p(2), hit(1) - p(1), hit(2) - p(2), 0, "k", "MaxHeadSize", 0.4);
            hUav = plot(p(1), p(2), "k+");
            hHit = plot(hit(1), hit(2), "kx");
            hold("off");
            legend([hUav, hHit], "UAV position", "Boresight-ground intersection", "Location", "northwest");
            xlabel("$x$ (m)");
            ylabel("$y$ (m)");
            cb = colorbar;
            cb.Label.String = "Sensor performance $s_n$";
            clim([0, 1]);

            % Full text width so it's included at 1:1 like the other figures;
            % 0.8 fits the square axes plus colorbar without side margins
            f = exportThesisFig(f, "sigmoidsensorexample", 0.8, Colormap="parula", Recolor=false, Resolution=300);
            close(f);
        end
        function antenna_patterns(tc)
            f = figure;
            xi = -90:0.1:90; % degrees
            n = [1, 4, 16, 64, 256];
            for ii = 1:length(n)
                gain = 10 .* log10(2 * (n(ii) + 1)) + 10 .* n(ii) .* log10(max(0, cosd(xi)));
                plot(xi, gain, "LineWidth", 2);
                if ii == 1
                    hold("on");
                end
            end
            hold("off");
            grid("on");
            ylim([-100, 35]);
            xlim([-90, 90]);
            xticks(-90:45:90);
            % exportThesisFig switches all text to the LaTeX interpreter
            title("Transmitting Antenna Gain Model");
            ylabel("Transmitting Antenna Gain (dBi)");
            xlabel("Elevation Angle from Antenna Boresight $\xi$ ($^\circ$)");
            legend("$\delta = " + string(n) + "$");

            exportThesisFig(f, "antennapatterns", 3/4);
            close(f);
        end
        function sigmoid_sensor_membership_functions(tc)
            s = [sigmoidSensor, sigmoidSensor];

            alphaDist = [2, 8];
            betaDist = [25, 1];
            alphaTilt = [50, 15]; % degrees
            betaTilt = [0.2, 3];

            s(1) = s(1).initialize(alphaDist(1), betaDist(1), alphaTilt(1), betaTilt(1));
            s(2) = s(2).initialize(alphaDist(2), betaDist(2), alphaTilt(2), betaTilt(2));

            set(groot, 'defaultTextInterpreter',          'latex');
            set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
            set(groot, 'defaultLegendInterpreter',        'latex');
            set(groot, 'defaultColorbarTickLabelInterpreter', 'latex');
            set(groot, 'defaultAxesFontSize', 10);   % = \footnotesize in a 12pt doc
            set(groot, 'defaultTextFontSize', 10);

            % Plot
            f = s(1).plotParameters;
            f = s(2).plotParameters(f);

            f = exportThesisFig(f, "sigmoidmodel", 3/4);
            close(f);
        end
    end

end
function f = plotSinrMap(x, y, SINR, sensors, positions, tx, cLimits)
    % Top-down SINR map for transmitting UAV tx, matching the sigmoid sensor
    % figure. Every UAV gets a marker, an arrow to the X where its boresight
    % meets the ground, and its -3 dB (half-power) beam footprint; the
    % transmitting UAV is drawn solid, the interferers dashed.
    f = figure;
    imagesc(x, y, SINR);
    set(gca, "YDir", "normal");   % y = 0 at the bottom
    axis("image");
    hold("on");
    for ii = 1:numel(sensors)
        [hit, ring] = beamFootprint(sensors{ii}, positions(ii, :));
        pos = positions(ii, :);
        if ii == tx
            style = "-";  marker = "k+";
        else
            style = "--"; marker = "ko";
        end
        hRing(ii) = plot(ring(:, 1), ring(:, 2), "k" + style); %#ok<AGROW>
        quiver(pos(1), pos(2), hit(1) - pos(1), hit(2) - pos(2), 0, "k", ...
            "LineStyle", style, "MaxHeadSize", 0.4);
        hHit = plot(hit(1), hit(2), "kx");
        hUav(ii) = plot(pos(1), pos(2), marker, "MarkerFaceColor", "w"); %#ok<AGROW>
    end
    hold("off");
    rx = setdiff(1:numel(sensors), tx);
    legend([hUav(tx), hUav(rx(1)), hHit, hRing(tx), hRing(rx(1))], ...
        "Transmitting UAV", "Interfering UAVs", "Boresight-ground intersection", ...
        "Transmitter $-3$ dB contour", "Interferer $-3$ dB contour", ...
        "Location", "southoutside", "NumColumns", 2);
    xlim([min(x), max(x)]);   % footprints can extend past the area
    ylim([min(y), max(y)]);
    xlabel("$x$ (m)");
    ylabel("$y$ (m)");
    cb = colorbar;
    cb.Label.String = "SINR (dB)";
    clim(cLimits);
end

function [hit, ring] = beamFootprint(sensor, pos)
    % Where the boresight meets the ground (z = 0), and the ground curve where
    % the gain is 3 dB below boresight: the intersection of the half-power cone
    % with the ground, as in rfSensor/plot. Azimuth 0 = +Y, 90 = +X.
    tlt = sensor.tilt;
    az = sensor.azimuth;
    hit = pos(1:2) + pos(3) * tand(tlt) * [sind(az), cosd(az)];

    % Rotate nadir to the boresight
    Ry = [cosd(tlt), 0, -sind(tlt); 0, 1, 0; sind(tlt), 0, cosd(tlt)];
    Rz = [sind(az), -cosd(az), 0; cosd(az), sind(az), 0; 0, 0, 1];
    R = Rz * Ry;
    b = R * [0; 0; -1];
    u = R * [1; 0; 0];
    v = R * [0; 1; 0];

    ha = sensor.halfAngle();
    phi = linspace(0, 2*pi, 720)';
    dirs = cosd(ha) .* b' + sind(ha) .* (cos(phi) .* u' + sin(phi) .* v');
    t = -pos(3) ./ dirs(:, 3);
    t(t <= 0) = NaN;   % cone generators that never reach the ground
    ring = pos(1:2) + t .* dirs(:, 1:2);
end
