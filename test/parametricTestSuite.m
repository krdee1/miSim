classdef parametricTestSuite < matlab.unittest.TestCase
    properties (Access = private)
        % System under test
        testClass = miSim;
        domain = rectangularPrism;
        obstacles = cell(1, 0);

        %% Diagnostic Parameters
        % No effect on simulation dynamics
        makeVideo = true; % disable video writing for big performance increase
        makePlots = true; % disable plotting for big performance increase (also disables video)
        plotCommsGeometry = false; % disable plotting communications geometries
        protectedRange = 0;
    end
    properties (TestParameter)
        %% Simulation Parameters
        timestep = num2cell([1]); % duration of one simulation timestep
        maxIter = num2cell([25]); % number of timesteps to run

        % Domain parameters
        minAlt = num2cell([1]); % minimum allowed agent altitude, make sure test cases don't conflict with this

        % Constraint parameters
        barrierGain = num2cell([100]);
        barrierExponent = num2cell([3]);

        % Sensing Objective Parameters
        sensorPerformanceMinimum = num2cell([1e-6]); % sensor performance threshhold for partition assignment
        discretizationStep = num2cell([0.01]); % sensing objective discretization step size
        % this value goes on to determine central differences used in
        % gradient ascent and partitioning element sizes

        % Agent Parameters
        collisionRadius = num2cell([0.1]);
        initialStepSize = num2cell([0.2]); % gradient ascent step size at the first iteration. Decreases linearly to 0 based on maxIter.

        % Sensor Model Parameters
        alphaDist = num2cell([2.5, 5]);
        betaDist = num2cell([3, 15]);
        alphaTilt = num2cell([15, 30]); % (degrees)methods
        betaTilt = num2cell([3, 15]);

        % Communications Parameters
        comRange = num2cell([3]);
    end

    methods (Test, ParameterCombination = "exhaustive")
        % Test cases
        function single_agent_gradient_ascent(tc, timestep, maxIter, barrierGain, barrierExponent, minAlt, sensorPerformanceMinimum, discretizationStep, collisionRadius, initialStepSize, alphaDist, betaDist, alphaTilt, betaTilt, comRange)
            % Set up square domain
            l = 10;
            tc.domain = tc.domain.initialize([zeros(1, 3); l * ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([.75 * l, 0.75 * l]), tc.domain, discretizationStep, tc.protectedRange, sensorPerformanceMinimum);
            
            % Set up agent
            sensorModel = sigmoidSensor;
            sensorModel = sensorModel.initialize(alphaDist, betaDist, alphaTilt, betaTilt);
            agentPos = [l/4, l/4, l/4];
            collisionGeometry = spherical;
            collisionGeometry = collisionGeometry.initialize(agentPos, collisionRadius, REGION_TYPE.COLLISION, "Agent 1 Collision Region");
            agents = {agent};
            agents{1} = agents{1}.initialize(agentPos, collisionGeometry, sensorModel, comRange, maxIter, initialStepSize, "Agent 1", tc.plotCommsGeometry);

            % Set up simulation
            tc.testClass = tc.testClass.initialize(tc.domain, agents, barrierGain, barrierExponent, minAlt, timestep, maxIter, tc.obstacles, tc.makePlots, tc.makeVideo);

            % Save simulation parameters to output file
            tc.testClass.writeParams();

            % Run
            tc.testClass = tc.testClass.run();

            % Cleanup
            tc.testClass.teardown();
        end
    end
end