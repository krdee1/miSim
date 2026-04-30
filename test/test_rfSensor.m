classdef test_rfSensor < matlab.unittest.TestCase
    properties (Access = private)
        % System under test
        testClass = sigmoidSensor;
    end

    methods (TestMethodSetup)
        function tc = setup(tc)
            % Reinitialize sensor with random parameters
            tc.testClass = rfSensor;
        end
    end

    methods (Test)
        function plot_RSS(tc)
            % Plot sensor performance with no sources of interference
            P_TX = 1e-3; % Transmit power (Watts)
            BW = 20e6; % Bandwidth (Hz)
            f_c = 2e9; % Center frequency (Hz)
            G_RX_dBi = 3; % Receiving Antenna Gain (dBi)

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c, G_RX_dBi, 0, 0);

            tc.testClass.plotParameters();
        end
        function plot_SNR(tc)
            % Plot sensor performance with no sources of interference
            P_TX = 1e-3; % Transmit power (Watts)
            BW = 20e6; % Bandwidth (Hz)
            f_c = 2e9; % Center frequency (Hz)
            G_RX_dBi = 3; % Receiving Antenna Gain (dBi)

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c, G_RX_dBi, 0, 0);

            altitude = 30;

            tc.testClass.plotPerformance(altitude);
        end
        function plot_SINR_one_interferer(tc)
            % Plot sensor performance with no sources of interference
            P_TX = 1e-3; % Transmit power (Watts)
            BW = 20e6; % Bandwidth (Hz)
            f_c = 2e9; % Center frequency (Hz)
            G_RX_dBi = 3; % Receiving Antenna Gain (dBi)

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c, G_RX_dBi, 0, 0);

            altitude = 30;
            otherSensorsPos = [6, -4, -1]; % relative to main sensor
            otherSensors = cell(1, 1);
            otherSensors{1} = tc.testClass; % One interfering sensor, identical to the main sensor

            tc.testClass.plotPerformance(altitude, otherSensorsPos, otherSensors);
        end

        function plot_SINR_heterogenous_interferers(tc)
            % Plot sensor performance with no sources of interference
            P_TX = 1e-3; % Transmit power (Watts)
            BW = 20e6; % Bandwidth (Hz)
            f_c = 2e9; % Center frequency (Hz)
            G_RX_dBi = 3; % Receiving Antenna Gain (dBi)

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c, G_RX_dBi, 0, 0);

            altitude = 30;
            otherSensorsPos = [6, -4, -1; -2, 6, 0]; % relative to main sensor
            otherSensors = cell(2, 1);
            otherSensors{1} = rfSensor; % two heterogenous interfering sensors
            otherSensors{2} = rfSensor;

            % Must use same center frequency and bandwidth for interference sources
            otherSensors{1} = otherSensors{1}.initialize(10 * P_TX, BW, f_c, G_RX_dBi, 0, 0);
            otherSensors{2} = otherSensors{2}.initialize(10 * P_TX, BW, f_c, G_RX_dBi, 0, 0);

            tc.testClass.plotPerformance(altitude, otherSensorsPos, otherSensors);
        end

        function plot_SINR_heterogenous_interferers_efficiently(tc)
            P_TX = 1e-3;
            BW = 20e6;
            f_c = 2e9;
            G_RX_dBi = 3;
            altitude = 30;

            sensor1 = rfSensor;
            sensor1 = sensor1.initialize(P_TX, BW, f_c, G_RX_dBi, 0, 0);
            sensor2 = rfSensor;
            sensor2 = sensor2.initialize(P_TX, BW, f_c, G_RX_dBi, 0, 0);
            sensor3 = rfSensor;
            sensor3 = sensor3.initialize(P_TX, BW, f_c, G_RX_dBi, 0, 0);

            pos1 = [0,  0,  altitude];
            pos2 = [6, -4,  altitude - 1];
            pos3 = [-2, 6,  altitude];

            % Build a shared target grid
            distances = -15:0.25:15;
            [Xg, Yg] = meshgrid(distances, distances);
            targetPos = [Xg(:), Yg(:), zeros(numel(Xg), 1)];

            % Call 1: cache empty, does all computations for this timestep
            [SINR1, ~, sensor1, others] = sensor1.sensorPerformance(pos1, targetPos, [pos2; pos3], {sensor2; sensor3});
            sensor2 = others{1};
            sensor3 = others{2};

            % Calls 2 and 3 use cached data
            [SINR2, ~, sensor2, others] = sensor2.sensorPerformance(pos2, targetPos, [pos1; pos3], {sensor1; sensor3});
            sensor1 = others{1};
            sensor3 = others{2};

            [SINR3, ~, sensor3, ~] = sensor3.sensorPerformance(pos3, targetPos, [pos1; pos2], {sensor1; sensor2});


            % All caches should be populated after the three calls
            tc.assertNotEmpty(sensor1.rssCache);
            tc.assertNotEmpty(sensor2.rssCache);
            tc.assertNotEmpty(sensor3.rssCache);

            % Plot SINR from each UAV's perspective
            sz = size(Xg);
            SINR1 = reshape(SINR1, sz);
            SINR2 = reshape(SINR2, sz);
            SINR3 = reshape(SINR3, sz);

            f = figure;
            tiledlayout(f, 1, 3, TileSpacing="compact", Padding="compact");

            nexttile;
            imagesc(distances, distances, SINR1); axis image; set(gca, YDir="normal"); hold on;
            scatter(pos1(1), pos1(2), 80, "g", "o", LineWidth=2);
            scatter([pos2(1), pos3(1)], [pos2(2), pos3(2)], 80, "r", "x", LineWidth=2);
            hold off; cb = colorbar; cb.Label.String = "SINR (dB)"; xlabel("X (m)"); ylabel("Y (m)"); title("SINR: UAV 1");

            nexttile;
            imagesc(distances, distances, SINR2); axis image; set(gca, YDir="normal"); hold on;
            scatter(pos2(1), pos2(2), 80, "g", "o", LineWidth=2);
            scatter([pos1(1), pos3(1)], [pos1(2), pos3(2)], 80, "r", "x", LineWidth=2);
            hold off; cb = colorbar; cb.Label.String = "SINR (dB)"; xlabel("X (m)"); ylabel("Y (m)"); title("SINR: UAV 2");

            nexttile;
            imagesc(distances, distances, SINR3); axis image; set(gca, YDir="normal"); hold on;
            scatter(pos3(1), pos3(2), 80, "g", "o", LineWidth=2);
            scatter([pos1(1), pos2(1)], [pos1(2), pos2(2)], 80, "r", "x", LineWidth=2);
            hold off; cb = colorbar; cb.Label.String = "SINR (dB)"; xlabel("X (m)"); ylabel("Y (m)"); title("SINR: UAV 3");
        end
    end
end