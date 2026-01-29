classdef experiments < matlab.unittest.TestCase
    methods (Test)
        function Test1(tc)
            s = scenario;

            % TODO - define test case in simulation in here and generate the inits on the fly as needed
            s = s.initialize("2026_01_28_16_11_39_miSimInits.mat");

            s = s.run();

            
            
        end
    end

end