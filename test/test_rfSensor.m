classdef test_rfSensor < matlab.unittest.TestCase
    properties (Access = private)
        % System under test
        testClass = sigmoidSensor;
    end

    methods (TestMethodSetup)
        function tc = setup(tc)
            % Reinitialize sensor with random parameters
            tc.testClass = rfSensor;
            % TODO
            tc.testClass = tc.testClass.initialize();
        end
    end

    methods (Test)
        % Test methods

        function test_SINR(tc)
            tc.testClass.plotParameters();
            % [SINR] = tc.testClass.sensorPerformance(obj, agentPos, agentPan, agentTilt, targetPos);
        end
    end

end