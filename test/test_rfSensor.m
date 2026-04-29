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

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c, G_RX_dBi);

            tc.testClass.plotParameters();
        end
        function plot_SNR(tc)
            % Plot sensor performance with no sources of interference
            P_TX = 1e-3; % Transmit power (Watts)
            BW = 20e6; % Bandwidth (Hz)
            f_c = 2e9; % Center frequency (Hz)
            G_RX_dBi = 3; % Receiving Antenna Gain (dBi)

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c, G_RX_dBi);

            altitude = 30;

            tc.testClass.plotPerformance(altitude);
        end
        function plot_SINR_one_interferer(tc)
            % Plot sensor performance with no sources of interference
            P_TX = 1e-3; % Transmit power (Watts)
            BW = 20e6; % Bandwidth (Hz)
            f_c = 2e9; % Center frequency (Hz)
            G_RX_dBi = 3; % Receiving Antenna Gain (dBi)

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c, G_RX_dBi);

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

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c, G_RX_dBi);

            altitude = 30;
            otherSensorsPos = [6, -4, -1; -2, 6, 0]; % relative to main sensor
            otherSensors = cell(2, 1);
            otherSensors{1} = rfSensor; % two heterogenous interfering sensors
            otherSensors{2} = rfSensor;

            % Must use same center frequency and bandwidth for interference sources
            otherSensors{1} = otherSensors{1}.initialize(10 * P_TX, BW, f_c, G_RX_dBi);
            otherSensors{2} = otherSensors{2}.initialize(100 * P_TX, BW, f_c, G_RX_dBi);

            tc.testClass.plotPerformance(altitude, otherSensorsPos, otherSensors);
        end
    end
end