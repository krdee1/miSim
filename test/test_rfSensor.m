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
            beamwidthExponent = 6;
            lossExponent = 2;

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c, G_RX_dBi, beamwidthExponent, 0, 0, lossExponent);

            tc.testClass.plotParameters();
        end
        function plot_SNR(tc)
            % Plot sensor performance with no sources of interference
            P_TX = 1e-3; % Transmit power (Watts)
            BW = 20e6; % Bandwidth (Hz)
            f_c = 2e9; % Center frequency (Hz)
            G_RX_dBi = 3; % Receiving Antenna Gain (dBi)
            beamwidthExponent = 6;
            lossExponent = 2;

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c, G_RX_dBi, beamwidthExponent, 30, 135, lossExponent);

            altitude = 30;

            % Boresight azimuth=135° (between +X at 90° and -Y at 180°) → hotspot at +X,-Y.
            % SNR at (5,-5) should be higher than at (5,+5).
            agentPos = [0, 0, altitude];
            [~, snrA] = tc.testClass.sensorPerformance(agentPos, [5, -5, 0]);
            % tc.testClass = tc.testClass.clearRssCache();
            [~, snrB] = tc.testClass.sensorPerformance(agentPos, [5,  5, 0]);
            tc.assertGreaterThan(snrA, snrB, "SNR should be higher toward boresight (+X,-Y) than away from it (+X,+Y)");

            tc.testClass.plotPerformance(altitude);
        end
        function plot_SINR_one_interferer(tc)
            % Plot sensor performance with no sources of interference
            P_TX = 1e-3; % Transmit power (Watts)
            BW = 20e6; % Bandwidth (Hz)
            f_c = 2e9; % Center frequency (Hz)
            G_RX_dBi = 3; % Receiving Antenna Gain (dBi)
            beamwidthExponent = 6;
            lossExponent = 2;

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c, G_RX_dBi, beamwidthExponent, 0, 0, lossExponent);

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
            beamwidthExponent = 6;
            lossExponent = 2;

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c, G_RX_dBi, beamwidthExponent, 0, 0, lossExponent);

            altitude = 30;
            otherSensorsPos = [6, -4, -1; -2, 6, 0]; % relative to main sensor
            otherSensors = cell(2, 1);
            otherSensors{1} = rfSensor; % two heterogenous interfering sensors
            otherSensors{2} = rfSensor;

            % Must use same center frequency and bandwidth for interference sources
            otherSensors{1} = otherSensors{1}.initialize(10 * P_TX, BW, f_c, G_RX_dBi, beamwidthExponent, 0, 0, lossExponent);
            otherSensors{2} = otherSensors{2}.initialize(10 * P_TX, BW, f_c, G_RX_dBi, beamwidthExponent, 0, 0, lossExponent);

            tc.testClass.plotPerformance(altitude, otherSensorsPos, otherSensors);
        end
        function plot_SINR_heterogenous_interferers_efficiently(tc)
            P_TX = 1e-3;
            BW = 20e6;
            f_c = 2e9;
            G_RX_dBi = 3;
            altitude = 30;
            beamwidthExponent = [6, 4, 10];
            lossExponent = 2;

            sensor1 = rfSensor;
            sensor1 = sensor1.initialize(P_TX, BW, f_c, G_RX_dBi, beamwidthExponent(1), 15, 45, lossExponent);
            sensor2 = rfSensor;
            sensor2 = sensor2.initialize(P_TX, BW, f_c, G_RX_dBi, beamwidthExponent(2), 10, 150, lossExponent);
            sensor3 = rfSensor;
            sensor3 = sensor3.initialize(P_TX, BW, f_c, G_RX_dBi, beamwidthExponent(3), 20, 200, lossExponent);

            pos1 = [0,  0,  altitude];
            pos2 = [6, -4,  altitude - 1];
            pos3 = [-2, 6,  altitude];

            % Build a shared target grid
            distances = -15:0.25:15;
            [Xg, Yg] = meshgrid(distances, distances);
            targetPos = [Xg(:), Yg(:), zeros(numel(Xg), 1)];

            % Call 1: cache empty, does all computations for this timestep
            [~, ~, sensor1, others] = sensor1.sensorPerformance(pos1, targetPos, [pos2; pos3], {sensor2; sensor3});
            sensor2 = others{1};
            sensor3 = others{2};

            % Calls 2 and 3 use cached data
            [~, ~, sensor2, others] = sensor2.sensorPerformance(pos2, targetPos, [pos1; pos3], {sensor1; sensor3});
            sensor1 = others{1};
            sensor3 = others{2};

            [~, ~, sensor3, ~] = sensor3.sensorPerformance(pos3, targetPos, [pos1; pos2], {sensor1; sensor2});

            % All caches should be populated after the three calls
            tc.assertNotEmpty(sensor1.rssCache);
            tc.assertNotEmpty(sensor2.rssCache);
            tc.assertNotEmpty(sensor3.rssCache);

            % Plot SINR from each UAV's perspective.
            % otherSensorsPos for plotPerformance: XY = offset from calling sensor, Z = absolute_alt - calling_alt.
            % This is exactly posOther - posSelf for each row.
            sensor1.plotPerformance(pos1(3), [pos2 - pos1; pos3 - pos1], {sensor2; sensor3});
            sensor2.plotPerformance(pos2(3), [pos1 - pos2; pos3 - pos2], {sensor1; sensor3});
            sensor3.plotPerformance(pos3(3), [pos1 - pos3; pos2 - pos3], {sensor1; sensor2});
        end
    end
end