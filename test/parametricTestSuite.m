classdef parametricTestSuite < matlab.unittest.TestCase
    properties (Access = private)
        % System under test
        testClass = miSim;
        domain = rectangularPrism;
        objective = sensingObjective;
        obstacles = cell(1, 0);

        %% Diagnostic Parameters
        % No effect on simulation dynamics
        timestep = 1;
        makeVideo = true; % disable video writing for big performance increase
        makePlots = true; % disable plotting for big performance increase (also disables video)
        plotCommsGeometry = false; % disable plotting communications geometries
        protectedRange = 0;
    end
    properties (TestParameter)
        %% Simulation Parameters
        maxIter = num2cell([200, 400]); % number of timesteps to run

        % Domain parameters
        minAlt = num2cell([1, 3]); % minimum allowed agent altitude, make sure test cases don't conflict with this

        % Sensing Objective Parameters
        discretizationStep = num2cell([0.01, 0.05]);

        % Agent Parameters
        collisionRange = num2cell([0.1, 0.5]);

        % Sensor Model Parameters
        betaDist = num2cell(3:6:15);
        betaTilt = num2cell(3:6:15);
        alphaDist = num2cell([2.5, 5]);
        alphaTilt = num2cell([15, 30]); % (degrees)

        % Communications Parameters
        comRange = num2cell(1:2:5);
    end

    methods (Test, ParameterCombination = "exhaustive")
        % Test methods

        function single_agent_gradient_ascent(tc, maxIter, minAlt, discretizationStep, collisionRange, alphaDist, alphaTilt, betaDist, betaTilt, comRange)
            1;
        end
    end

end