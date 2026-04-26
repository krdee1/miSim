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
            otherSensors{1} = tc.testClass; % 2 identical sensors

            tc.testClass.plotPerformance(altitude, otherSensorsPos, otherSensors);
            
        end
    end
end