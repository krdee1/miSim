classdef test_sigmoidSensor < matlab.unittest.TestCase
    properties (Access = private)
        % System under test
        testClass = sigmoidSensor;

        % Domain
        domain = rectangularPrism;

        % Sensor parameter ranges
        betaDistMin = 3;
        betaDistMax = 15;
        betaTiltMin = 3;
        betaTiltMax = 15;
        alphaDistMin = 2.5;
        alphaDistMax = 3;
        alphaTiltMin = 15; % degrees
        alphaTiltMax = 30; % degrees
    end

    methods (TestMethodSetup)
        function tc = setup(tc)
            % Reinitialize sensor with random parameters
            tc.testClass = sigmoidSensor;
            tc.testClass = tc.testClass.initialize(tc.alphaDistMin + rand * (tc.alphaDistMax - tc.alphaDistMin), tc.betaDistMin + rand * (tc.betaDistMax - tc.betaDistMin), tc.alphaTiltMin + rand * (tc.alphaTiltMax - tc.alphaTiltMin), tc.betaTiltMin + rand * (tc.betaTiltMax - tc.betaTiltMin));
        end
    end

    methods (Test)
        % Test methods
        function test_sensorPerformance(tc)
            tc.testClass = sigmoidSensor;
            alphaDist = 2.5;
            betaDist = 3;
            alphaTilt = 15; % degrees
            betaTilt = 3;
            h = 1e-6;
            tc.testClass = tc.testClass.initialize(alphaDist, betaDist, alphaTilt, betaTilt);
            
            % Plot (optional)
            % tc.testClass.plotParameters();

            % Anticipate perfect performance for a point directly below and
            % extremely close
            tc.verifyEqual(tc.testClass.sensorPerformance([0, 0, h], 0, [0, 0, 0]), 1, 'RelTol', 1e-3); 
            % It looks like mu_t can max out at really low values like 0.37
            % when alphaTilt and betaTilt are small, which seems wrong

            % Performance at nadir point, distance alphaDist should be 1/2 exactly
            tc.verifyEqual(tc.testClass.sensorPerformance([0, 0, alphaDist], 0, [0, 0, 0]), 1/2);

            % Performance at (almost) 0 distance, alphaTilt should be 1/2
            tc.verifyEqual(tc.testClass.sensorPerformance([0, 0, h], 0, [tand(alphaTilt)*h, 0, 0]), 1/2, 'RelTol', 1e-3);

            % Performance at great distance should be 0
            tc.verifyEqual(tc.testClass.sensorPerformance([0, 0, 10], 0, [0, 0, 0]), 0, 'AbsTol', 1e-9);

            % Performance at great tilt should be 0
            tc.verifyEqual(tc.testClass.sensorPerformance([0, 0, h], 0, [5, 5, 0]), 0, 'AbsTol', 1e-9);
        end
    end

end