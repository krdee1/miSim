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
        alphaTiltMin = deg2rad(15);
        alphaTiltMax = deg2rad(30);
    end

    methods (TestMethodSetup)
        function tc = setup(tc)
            % Reinitialize sensor with random parameters
            tc.testClass = sigmoidSensor;
            tc.testClass = tc.testClass.initialize(tc.alphaDistMin + rand * (tc.alphaDistMax - tc.alphaDistMin), tc.betaDistMin + rand * (tc.betaDistMax - tc.betaDistMin), NaN, NaN, tc.alphaTiltMin + rand * (tc.alphaTiltMax - tc.alphaTiltMin), tc.betaTiltMin + rand * (tc.betaTiltMax - tc.betaTiltMin));
        end
    end

    methods (Test)
        % Test methods
        function test_sensorPerformance(tc)
            tc.testClass = sigmoidSensor;
            alphaDist = 2.5;
            betaDist = 3;
            alphaTilt = deg2rad(15);
            betaTilt = 3;
            tc.testClass = tc.testClass.initialize(alphaDist, betaDist, NaN, NaN, alphaTilt, betaTilt);
            
            % Plot
            tc.testClass.plotParameters();

            % Performance at current position should be maximized (1)
            % some wiggle room is needed for certain parameter conditions,
            % e.g. small alphaDist and betaDist produce mu_d slightly < 1
            tc.verifyEqual(tc.testClass.sensorPerformance(zeros(1, 3), NaN, 0, zeros(1, 3)), 1, 'AbsTol', 1e-3); 
            % It looks like mu_t can max out at really low values like 0.37
            % when alphaTilt and betaTilt are small, which seems wrong

            % Performance at distance alphaDist should be 1/2
            tc.verifyEqual(tc.testClass.sensorPerformance([0, 0, alphaDist], NaN, 0, [0, 0, 0]), 1/2, 'AbsTol', 1e-3);
        end
    end

end