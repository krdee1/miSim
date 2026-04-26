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
        function plot_SNR(tc)
            % Plot sensor performance with no sources of interference
            P_TX = 1e-3; % Transmit power (Watts)
            BW = 20e6; % Bandwidth (Hz)
            f_c = 2e9; % Center frequency (Hz)
            G_RX_dBi = 3; % Receiving Antenna Gain (dBi)

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c, G_RX_dBi);

            tc.testClass.plotParameters();
        end
    end
end