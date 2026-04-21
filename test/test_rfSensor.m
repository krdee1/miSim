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

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c);

            tc.testClass.plotParameters();
        end
        function plot_SINR(tc)
            % Plot sensor performance with a single source of interference
            P_TX = 1e-3; % Transmit power (Watts)
            BW = 20e6; % Bandwidth (Hz)
            f_c = 2e9; % Center frequency (Hz)

            tc.testClass = tc.testClass.initialize(P_TX, BW, f_c);

            
    end
end